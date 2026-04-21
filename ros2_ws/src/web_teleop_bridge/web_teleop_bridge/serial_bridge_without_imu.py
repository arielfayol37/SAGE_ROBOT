#!/usr/bin/env python3
# serial_bridge_without_imu.py
#
# Fault-tolerant ROS 2 <-> MCU serial bridge with auto-reconnect.
#
# Host -> MCU  : 0x78 + <ff>                       # v, w
# MCU  -> Host : [0x7E][TYPE][LEN][<payload>]
#   TYPE=0x02 -> odom     payload x, y, theta, v, w  (20 bytes, 5x float32)
#   TYPE=0x04 -> battery  payload voltage            ( 4 bytes, 1x float32)
#
# Publishes:
#   /odom                  nav_msgs/Odometry
#   /battery_state         sensor_msgs/BatteryState
#   odom -> base_link      TF

import glob
import math
import struct
import threading
import time

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

# --- Protocol constants -------------------------------------------------------
SOF          = 0x7E
TYPE_ODOM    = 0x02
TYPE_BATTERY = 0x04

# Pre-compiled struct objects (faster than passing a format string every call)
_S_ODOM    = struct.Struct('<fffff')    # x, y, th, v, w
_S_BATTERY = struct.Struct('<f')        # voltage
_S_TX      = struct.Struct('<Bff')      # 0x78, v, w

LEN_ODOM    = _S_ODOM.size              # 20
LEN_BATTERY = _S_BATTERY.size           # 4
LEN_TX      = _S_TX.size                # 9

# --- Reconnect policy ---------------------------------------------------------
_RECONNECT_MIN_S = 0.25
_RECONNECT_MAX_S = 5.0
_RECONNECT_WARN_PERIOD_S = 5.0

_NAN = float('nan')


class SerialBridgeNoImu(Node):
    def __init__(self):
        super().__init__('serial_bridge_no_imu')

        # ---- Parameters ------------------------------------------------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')
        self.declare_parameter('serial_timeout_s', 0.05)
        self.declare_parameter('rx_chunk', 256)
        self.declare_parameter('battery_topic', '/battery_state')
        # Percentage estimate is skipped when min >= max.
        self.declare_parameter('battery_v_min', 0.0)
        self.declare_parameter('battery_v_max', 0.0)
        self.declare_parameter('battery_cell_count', 0)   # 0 = unknown
        self.declare_parameter('auto_detect_port', True)  # rescan /dev/ttyACM* on (re)connect

        self._configured_port  = str(self.get_parameter('port').value)
        self._baud             = int(self.get_parameter('baud').value)
        self._cmd_topic        = str(self.get_parameter('cmd_topic').value)
        self._odom_topic       = str(self.get_parameter('odom_topic').value)
        self._battery_topic    = str(self.get_parameter('battery_topic').value)
        self._frame_odom       = str(self.get_parameter('frame_odom').value)
        self._frame_base       = str(self.get_parameter('frame_base').value)
        self._serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self._rx_chunk         = int(self.get_parameter('rx_chunk').value)
        self._battery_v_min    = float(self.get_parameter('battery_v_min').value)
        self._battery_v_max    = float(self.get_parameter('battery_v_max').value)
        self._battery_cells    = int(self.get_parameter('battery_cell_count').value)
        self._auto_detect      = bool(self.get_parameter('auto_detect_port').value)

        # ---- Serial state ----------------------------------------------------
        self._serial_lock = threading.Lock()
        self.ser = None
        self._current_port = None

        # Covariance templates are built once and re-assigned per message.
        self._pose_cov_template  = self._build_cov([1e-3, 1e-3, 1e3, 1e3, 1e3, 5e-3])
        self._twist_cov_template = self._build_cov([5e-3, 1e3, 1e3, 1e3, 1e3, 5e-3])

        # ---- ROS I/O ---------------------------------------------------------
        self.create_subscription(Twist, self._cmd_topic, self.on_cmd_vel, 10)
        self.odom_pub       = self.create_publisher(Odometry,    self._odom_topic,    10)
        self.battery_pub    = self.create_publisher(BatteryState, self._battery_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- RX / reconnect worker ------------------------------------------
        self._shutdown = threading.Event()
        self._buf = bytearray()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name='serial_rx',
            daemon=True,
        )
        self._rx_thread.start()

        self.get_logger().info(
            f"TX:{self._cmd_topic} | RX->ODOM:{self._odom_topic} | "
            f"Frames: odom='{self._frame_odom}' base='{self._frame_base}'"
        )

    # --------------------------------------------------------------------------
    # Serial connection management
    # --------------------------------------------------------------------------

    @staticmethod
    def _build_cov(diag):
        """Row-major 36-element covariance list with the given 6-element diagonal."""
        cov = [0.0] * 36
        for i, v in enumerate(diag):
            cov[i * 7] = float(v)   # i * 6 + i == i * 7
        return cov

    def _candidate_ports(self):
        """Ports to try, in priority order. Detected ACM ports first, then configured."""
        seen = []
        if self._auto_detect:
            for p in sorted(glob.glob('/dev/ttyACM*')):
                if p not in seen:
                    seen.append(p)
        if self._configured_port and self._configured_port not in seen:
            seen.append(self._configured_port)
        return seen

    def _close_serial_unlocked(self):
        """Close current serial handle. Caller must hold _serial_lock."""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._current_port = None

    def _open_serial(self):
        """Try every candidate port. Returns True on success. Never raises."""
        with self._serial_lock:
            self._close_serial_unlocked()

            for port in self._candidate_ports():
                ser = self._try_open(port)
                if ser is None:
                    continue

                # Let USB CDC settle and discard any stale bytes.
                time.sleep(0.3)
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass

                self.ser = ser
                self._current_port = port
                self.get_logger().info(f"Serial connected: {port}@{self._baud}")
                return True

        return False

    def _try_open(self, port):
        """Open one port. Returns the Serial handle or None."""
        kwargs = dict(
            port=port,
            baudrate=self._baud,
            timeout=self._serial_timeout_s,
            write_timeout=0.2,
        )
        try:
            # 'exclusive' needs pyserial >= 3.3.
            return serial.Serial(exclusive=True, **kwargs)
        except TypeError:
            try:
                return serial.Serial(**kwargs)
            except Exception as e:
                self.get_logger().debug(f"Open {port} failed: {e}")
                return None
        except Exception as e:
            self.get_logger().debug(f"Open {port} failed: {e}")
            return None

    def _mark_connection_lost(self):
        """Mark the port as dead. RX loop will reconnect."""
        with self._serial_lock:
            had_port = self.ser is not None
            prev_port = self._current_port
            self._close_serial_unlocked()
        if had_port:
            self.get_logger().warn(f"Serial connection lost on {prev_port}")

    # --------------------------------------------------------------------------
    # TX (cmd_vel)
    # --------------------------------------------------------------------------

    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Enforce a minimum usable turn speed when essentially stationary.
        min_inplace_w = 0.7
        stationary_v_thresh = 0.05
        if abs(v) < stationary_v_thresh and 0.0 < abs(w) < min_inplace_w:
            w = min_inplace_w if w > 0.0 else -min_inplace_w

        pkt = _S_TX.pack(0x78, v, w)

        with self._serial_lock:
            ser = self.ser
            if ser is None:
                return  # Not connected; RX thread owns reconnection.
            try:
                n = ser.write(pkt)
                if n != LEN_TX:
                    self.get_logger().warn(f"Short serial write {n}/{LEN_TX}")
                    return
            except (serial.SerialException, OSError, serial.SerialTimeoutException) as e:
                self.get_logger().error(f"Serial write failed: {e}")
                self._close_serial_unlocked()   # already holding the lock
            except Exception as e:
                self.get_logger().error(f"Unexpected serial write error: {e}")
                self._close_serial_unlocked()

    # --------------------------------------------------------------------------
    # RX + reconnect loop
    # --------------------------------------------------------------------------

    def _rx_loop(self):
        buf = self._buf
        backoff = _RECONNECT_MIN_S
        last_warn_t = 0.0

        # Hot-path local bindings
        sof          = SOF
        type_odom    = TYPE_ODOM
        type_battery = TYPE_BATTERY
        len_odom     = LEN_ODOM
        len_battery  = LEN_BATTERY
        buf_index    = buf.index
        odom_unpack  = _S_ODOM.unpack_from
        bat_unpack   = _S_BATTERY.unpack_from

        while not self._shutdown.is_set():
            ser = self.ser
            if ser is None:
                # Reconnect path
                if self._open_serial():
                    buf.clear()
                    backoff = _RECONNECT_MIN_S
                    last_warn_t = 0.0
                    continue

                now = time.monotonic()
                if now - last_warn_t > _RECONNECT_WARN_PERIOD_S:
                    self.get_logger().warn(
                        f"No serial port available; retrying (backoff={backoff:.2f}s)"
                    )
                    last_warn_t = now
                if self._shutdown.wait(backoff):
                    break
                backoff = min(backoff * 2.0, _RECONNECT_MAX_S)
                continue

            # Read
            try:
                chunk = ser.read(self._rx_chunk)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"Serial read error (disconnected?): {e}")
                self._mark_connection_lost()
                continue
            except Exception as e:
                self.get_logger().warn(f"Unexpected serial read error: {e}")
                time.sleep(0.05)
                continue

            if chunk:
                buf.extend(chunk)

            # Parse frames from the buffer
            while True:
                try:
                    i = buf_index(sof)
                except ValueError:
                    # No SOF anywhere; drop everything
                    buf.clear()
                    break

                # Need SOF + TYPE + LEN
                if len(buf) - i < 3:
                    if i > 0:
                        del buf[:i]
                    break

                typ = buf[i + 1]
                ln  = buf[i + 2]
                frame_len = 3 + ln

                # Wait for full frame
                if len(buf) - i < frame_len:
                    if i > 0:
                        del buf[:i]
                    break

                # Dispatch without copying the payload
                payload_off = i + 3
                if typ == type_odom and ln == len_odom:
                    try:
                        x, y, th, v, w = odom_unpack(buf, payload_off)
                    except struct.error:
                        pass
                    else:
                        self._publish_odom(x, y, th, v, w)
                elif typ == type_battery and ln == len_battery:
                    try:
                        (voltage,) = bat_unpack(buf, payload_off)
                    except struct.error:
                        pass
                    else:
                        self._publish_battery(voltage)
                # else: unknown / malformed frame — silently skipped

                del buf[:i + frame_len]

    # --------------------------------------------------------------------------
    # Message publishers
    # --------------------------------------------------------------------------

    def _publish_odom(self, x, y, th, v, w):
        # Yaw-only quaternion: qx = qy = 0, qz = sin(th/2), qw = cos(th/2)
        half = th * 0.5
        qz = math.sin(half)
        qw = math.cos(half)

        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp    = now
        od.header.frame_id = self._frame_odom
        od.child_frame_id  = self._frame_base

        p = od.pose.pose
        p.position.x = x
        p.position.y = y
        # position.z defaults to 0.0
        p.orientation.z = qz
        p.orientation.w = qw
        # orientation.x / .y default to 0.0

        t = od.twist.twist
        t.linear.x  = v
        t.angular.z = w

        od.pose.covariance  = self._pose_cov_template
        od.twist.covariance = self._twist_cov_template

        self.odom_pub.publish(od)

        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self._frame_odom
        tf.child_frame_id  = self._frame_base
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _publish_battery(self, voltage):
        msg = BatteryState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_base
        msg.voltage = float(voltage)

        # Unknowns per REP-140: NaN, not 0.0
        msg.temperature     = _NAN
        msg.current         = _NAN
        msg.charge          = _NAN
        msg.capacity        = _NAN
        msg.design_capacity = _NAN

        if self._battery_v_max > self._battery_v_min:
            span = self._battery_v_max - self._battery_v_min
            pct = (voltage - self._battery_v_min) / span
            if pct < 0.0:
                pct = 0.0
            elif pct > 1.0:
                pct = 1.0
            msg.percentage = float(pct)
        else:
            msg.percentage = _NAN

        msg.power_supply_status     = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True

        if self._battery_cells > 0:
            per_cell = voltage / self._battery_cells
            msg.cell_voltage     = [per_cell] * self._battery_cells
            msg.cell_temperature = [_NAN]     * self._battery_cells
        else:
            msg.cell_voltage     = []
            msg.cell_temperature = []

        self.battery_pub.publish(msg)

    # --------------------------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------------------------

    def destroy_node(self):
        self._shutdown.set()
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        with self._serial_lock:
            self._close_serial_unlocked()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridgeNoImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
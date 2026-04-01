import time
import math
import usb.core
import usb.util
import struct

VID = 0x2886
PID = 0x001A  # XVF3800 USB 4-mic array

# From Seeed/host docs:
# AEC_AZIMUTH_VALUES: (resid=33, cmdid=75) returns 4 floats (radians).
AEC_AZIMUTH_RESID = 33
AEC_AZIMUTH_CMDID = 0x80 | 75  # read command = 0x80 | cmdid
NUM_FLOATS = 4
STATUS_PLUS_DATA_LEN = 1 + 4 * NUM_FLOATS

TIMEOUT_MS = 100000

def read_aec_azimuth_values(dev):
    resp = dev.ctrl_transfer(
        usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
        0,
        AEC_AZIMUTH_CMDID,
        AEC_AZIMUTH_RESID,
        STATUS_PLUS_DATA_LEN,
        TIMEOUT_MS
    )
    # resp[0] is status; remaining bytes are 4 floats little-endian
    data_bytes = resp.tobytes()[1:1 + 4 * NUM_FLOATS]
    vals = struct.unpack("<ffff", data_bytes)
    return vals  # radians

def main():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError("XVF3800 device not found (VID=0x2886, PID=0x001A).")

    # Some systems need this:
    try:
        if dev.is_kernel_driver_active(3):
            dev.detach_kernel_driver(3)
    except Exception:
        pass

    while True:
        beam1, beam2, free_run, auto_sel = read_aec_azimuth_values(dev)
        doa_deg = (math.degrees(auto_sel) + 360.0) % 360.0
        print(f"DOA (auto-selected beam): {doa_deg:.2f}°")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
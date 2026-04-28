#!/usr/bin/env python3
"""Republishes the AprilTag dock_tag TF as a PoseStamped on /detected_dock_pose."""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros


class DockPosePublisher(Node):
    def __init__(self):
        super().__init__('dock_pose_publisher')
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('tag_frame', 'dock_tag')
        self.declare_parameter('publish_hz', 15.0)

        self.source_frame = self.get_parameter('source_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value
        hz = float(self.get_parameter('publish_hz').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.create_timer(1.0 / hz, self.tick)

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame, self.tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return

        msg = PoseStamped()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = self.source_frame
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(DockPosePublisher())
    rclpy.shutdown()

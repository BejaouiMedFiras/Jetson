#!/usr/bin/env python3
"""
scan_relay.py
Bridges YDLIDAR BEST_EFFORT output to RELIABLE so slam_toolbox connects.

Topic flow:
  /scan_raw  (BEST_EFFORT, published by ydlidar driver)
      ↓  this node
  /scan      (RELIABLE, consumed by slam_toolbox)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


BEST_EFFORT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self._pub = self.create_publisher(LaserScan, '/scan', RELIABLE_QOS)
        self._sub = self.create_subscription(
            LaserScan, '/scan_raw', self._cb, BEST_EFFORT_QOS)
        self.get_logger().info(
            'scan_relay ready: /scan_raw (BEST_EFFORT) → /scan (RELIABLE)')

    def _cb(self, msg: LaserScan):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

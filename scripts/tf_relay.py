#!/usr/bin/env python3
"""
tf_relay.py

Relays /tf to /{ns}/tf for slam_toolbox's map→odom transform only.

The Pi bringup (diff_drive_controller) now publishes its dynamic TFs
(odom→base_footprint, etc.) DIRECTLY to /{ns}/tf — no relay is needed
for hardware TFs.

This relay exists solely to bridge slam_toolbox's map→odom transform.
slam_toolbox publishes to the root /tf topic regardless of configuration.
Nav2 (namespaced) listens on /{ns}/tf. This script bridges the gap.

Why not topic_tools relay?
  topic_tools relay uses VOLATILE / VOLATILE QoS. The Pi bringup's
  diff_drive_controller publishes dynamic TFs with BEST_EFFORT. This
  script uses explicit QoS profiles to match both sides correctly:
    - Sub:  VOLATILE / BEST_EFFORT  (matches slam_toolbox publisher)
    - Pub:  VOLATILE / RELIABLE     (matches Nav2 subscriber expectation)

Why NOT TRANSIENT_LOCAL here (unlike map_relay.py)?
  Dynamic transforms must never be buffered with TRANSIENT_LOCAL. A late-
  joining subscriber receiving a stale dynamic TF (e.g. an old map→odom
  transform) would cause incorrect localisation. Dynamic TFs are published
  continuously and subscribers catch up naturally within one cycle.

Usage: python3 tf_relay.py <target_topic>
  e.g. python3 tf_relay.py /tb3_1/tf
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self, target_topic: str):
        super().__init__('tf_relay')

        # Subscribe BEST_EFFORT to match slam_toolbox's publisher QoS.
        # Publish RELIABLE so Nav2 nodes always receive every message.
        volatile_sub_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        volatile_pub_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(TFMessage, target_topic, volatile_pub_qos)
        self.sub = self.create_subscription(
            TFMessage, '/tf', self.callback, volatile_sub_qos
        )

        self.get_logger().info(
            f'tf_relay: /tf → {target_topic} '
            f'(slam_toolbox map→odom only; hardware TFs published directly by Pi)'
        )

    def callback(self, msg: TFMessage):
        self.pub.publish(msg)


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else '/tf_relay_out'
    rclpy.init()
    node = TFRelay(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
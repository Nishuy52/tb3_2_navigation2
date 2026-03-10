#!/usr/bin/env python3
"""
tf_relay.py

Relays /tf to /{ns}/tf, forwarding dynamic transforms from the robot's
hardware bringup (which publishes to root /tf) into the namespaced topic
that Nav2 listens on.

Why not topic_tools relay?
  topic_tools relay uses VOLATILE QoS. While dynamic TFs are published
  continuously (so missed messages recover quickly), the relay also has
  no awareness of QoS mismatches between publisher and subscriber. This
  script uses explicit QoS profiles matching what Nav2 expects.

Why NOT TRANSIENT_LOCAL here (unlike tf_static_relay.py)?
  Dynamic transforms must NOT be buffered with TRANSIENT_LOCAL. A late-
  joining subscriber receiving a stale dynamic TF (e.g. an old odom→base
  transform) would cause incorrect localisation. Dynamic TFs are published
  at high frequency and subscribers catch up naturally within one cycle.

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

        # Dynamic TF uses VOLATILE / BEST_EFFORT on the publisher side
        # (robot bringup) and VOLATILE / RELIABLE on the Nav2 subscriber side.
        # We publish with RELIABLE so Nav2 nodes always receive every message.
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
            f'tf_relay: /tf → {target_topic} (VOLATILE, not buffered)'
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
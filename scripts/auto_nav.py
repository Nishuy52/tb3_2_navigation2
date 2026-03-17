#!/usr/bin/env python3
"""
auto_nav.py

Frontier-based autonomous exploration node for TurtleBot3.

Phase 1 — Bootstrap: drives a small 3-leg open square (~0.5 m sides) via
Nav2 NavigateToPose so that SLAM builds an initial map before exploration
begins. Nav2 can navigate in SLAM mode even without a prior map because the
global costmap uses a fixed 20×20 m grid populated only by the obstacle_layer.

Phase 2 — Explore: detects frontier cells (free cells adjacent to unknown
space), clusters them, and sends NavigateToPose goals to Nav2 to explore.
Stops when the fraction of unknown cells drops below `unknown_threshold`.

Launch:
    ros2 launch tb3_2_navigation2 auto_nav.launch.py
    ros2 launch tb3_2_navigation2 auto_nav.launch.py namespace:=tb3_2 unknown_threshold:=0.05

Parameters:
    namespace          (str,   default 'tb3_2') - must match running Nav2 instance
    unknown_threshold  (float, default 0.05)    - stop when unknown ratio < this value
"""

import math
import time
from enum import Enum, auto

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

# Frontier clustering parameters
_CLUSTER_RADIUS_M  = 0.5   # max distance to merge a cell into an existing cluster (metres)
_MIN_CLUSTER_CELLS = 3     # discard clusters smaller than this (noise filter)

# Stuck / blacklist parameters
_STUCK_GOAL_DIST_M   = 0.30  # same-goal detection radius (metres)
_ABORT_THRESHOLD     = 1     # aborts before blacklisting (1 = immediate on first abort)
_SAME_GOAL_THRESHOLD = 2     # consecutive same-goal re-selections before blacklisting
_BLACKLIST_RADIUS_M  = 0.5   # radius around a blacklisted point to suppress (metres)

# Minimum distance the robot must travel to the goal to guarantee a SLAM map update.
# Must be comfortably above slam_toolbox's minimum_travel_distance (0.1 m).
_MIN_GOAL_DIST_M = 0.5

# Post-success map-settle delay
_MAP_SETTLE_S = 3.0  # seconds to wait after reaching a frontier before re-planning

# Bootstrap square side length
_BOOTSTRAP_SIDE_M = 0.5


class _Phase(Enum):
    BOOTSTRAP = auto()
    EXPLORE   = auto()


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('namespace', 'tb3_2')
        self.declare_parameter('unknown_threshold', 0.05)

        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self._threshold = self.get_parameter('unknown_threshold').get_parameter_value().double_value

        if self._threshold > 0.20:
            self.get_logger().warn(
                f'unknown_threshold={self._threshold:.0%} is high; '
                f'exploration may stop prematurely.'
            )

        # ── Derived names ────────────────────────────────────────────────────
        self._map_frame  = f'{ns}/map'
        self._base_frame = f'{ns}/base_footprint'
        map_topic        = f'/{ns}/map'
        action_name      = f'/{ns}/navigate_to_pose'

        self.get_logger().info(
            f'FrontierExplorer: ns={ns!r}, map={map_topic!r}, '
            f'action={action_name!r}, threshold={self._threshold:.1%}'
        )

        # ── QoS (matches map_relay.py — TRANSIENT_LOCAL so we get the map
        #   even if slam_toolbox published before we started) ─────────────────
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── Subscriptions / TF ──────────────────────────────────────────────
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, transient_qos
        )
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Action client ────────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, action_name)

        # ── State ────────────────────────────────────────────────────────────
        self._current_map: OccupancyGrid | None = None
        self._navigating  = False
        self._last_goal: tuple[float, float] | None = None

        # Bootstrap phase
        self._phase = _Phase.BOOTSTRAP
        self._bootstrap_legs: list[tuple[float, float]] = []
        self._bootstrap_idx   = 0

        # Post-success settle
        self._map_settle_until = 0.0

        # Stuck tracking — two separate counters
        self._abort_count     = 0   # consecutive aborted goals
        self._same_goal_count = 0   # consecutive same-goal re-selections

        self._blacklist: list[tuple[float, float]] = []
        self._current_goal_handle = None

        # ── Exploration timer (fires every 0.5 s) ────────────────────────────
        self._explore_timer = self.create_timer(0.5, self._explore_once)

    # ── Map callback ─────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        self._current_map = msg
        if self._phase == _Phase.EXPLORE and self._check_stop_condition(msg):
            self._explore_timer.cancel()

    # ── Stop condition ───────────────────────────────────────────────────────

    def _check_stop_condition(self, msg: OccupancyGrid) -> bool:
        data  = np.frombuffer(msg.data, dtype=np.int8)
        total = len(data)
        if total == 0:
            return False
        ratio = float(np.sum(data == -1)) / total
        if ratio < self._threshold:
            self.get_logger().info(
                f'Exploration complete: unknown ratio {ratio:.1%} < '
                f'threshold {self._threshold:.1%}. Stopping.'
            )
            return True
        return False

    # ── Robot pose ───────────────────────────────────────────────────────────

    def _get_robot_pose(self) -> tuple[float, float] | None:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    # ── Bootstrap phase ──────────────────────────────────────────────────────

    def _run_bootstrap(self):
        """Drive a 3-leg open square to give SLAM an initial map."""
        if self._navigating:
            return

        # If a map is already available, skip bootstrap entirely
        if self._current_map is not None:
            self.get_logger().info('Map already available. Skipping bootstrap.')
            if self._navigating and self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._navigating = False
            self._phase = _Phase.EXPLORE
            return

        # Compute waypoints on the first successful TF lookup
        if not self._bootstrap_legs:
            pose = self._get_robot_pose()
            if pose is None:
                return  # TF not ready yet, retry next timer tick
            rx0, ry0 = pose
            s = _BOOTSTRAP_SIDE_M
            self._bootstrap_legs = [
                (rx0 + s, ry0),      # forward
                (rx0 + s, ry0 - s),  # right
                (rx0,     ry0 - s),  # back-left (open square, no return leg)
            ]
            self.get_logger().info(
                f'Bootstrap: driving 3-leg square from ({rx0:.2f}, {ry0:.2f})'
            )

        if self._bootstrap_idx >= len(self._bootstrap_legs):
            self.get_logger().info('Bootstrap complete. Starting frontier exploration.')
            self._phase = _Phase.EXPLORE
            return

        leg = self._bootstrap_legs[self._bootstrap_idx]
        self.get_logger().info(
            f'Bootstrap leg {self._bootstrap_idx + 1}/{len(self._bootstrap_legs)}: '
            f'({leg[0]:.2f}, {leg[1]:.2f})'
        )
        self._send_goal(*leg)

    # ── Frontier detection ───────────────────────────────────────────────────

    def _find_frontiers(self, msg: OccupancyGrid) -> list[tuple[float, float]]:
        """Return world-coordinate centroids of frontier clusters."""
        info = msg.info
        if info.width == 0 or info.height == 0:
            return []

        grid = np.frombuffer(msg.data, dtype=np.int8).reshape(
            (info.height, info.width)
        )

        free    = (grid == 0)
        unknown = (grid == -1)

        # Adjacent-to-unknown mask (4-connected), eliminating border wrap
        adj_unknown = (
            np.roll(unknown,  1, axis=0) |
            np.roll(unknown, -1, axis=0) |
            np.roll(unknown,  1, axis=1) |
            np.roll(unknown, -1, axis=1)
        )
        adj_unknown[[0, -1], :] = False
        adj_unknown[:, [0, -1]] = False

        frontier_mask = free & adj_unknown
        indices = np.argwhere(frontier_mask)  # shape (N, 2): [[row, col], ...]

        if len(indices) == 0:
            return []

        # ── Greedy distance-based clustering ─────────────────────────────────
        radius_cells    = _CLUSTER_RADIUS_M / info.resolution
        radius_cells_sq = radius_cells ** 2

        clusters: list[list[np.ndarray]] = []
        cluster_means: list[np.ndarray]  = []

        for pt in indices:
            best_idx     = -1
            best_dist_sq = radius_cells_sq + 1
            for i, mean in enumerate(cluster_means):
                d2 = float(np.sum((pt - mean) ** 2))
                if d2 < best_dist_sq:
                    best_dist_sq = d2
                    best_idx     = i
            if best_idx >= 0:
                clusters[best_idx].append(pt)
                n = len(clusters[best_idx])
                cluster_means[best_idx] += (pt - cluster_means[best_idx]) / n
            else:
                clusters.append([pt])
                cluster_means.append(pt.astype(float))

        # ── Convert surviving clusters to world coordinates ───────────────────
        centroids: list[tuple[float, float]] = []
        ox  = info.origin.position.x
        oy  = info.origin.position.y
        res = info.resolution

        for cluster, mean in zip(clusters, cluster_means):
            if len(cluster) < _MIN_CLUSTER_CELLS:
                continue
            row, col = mean[0], mean[1]
            wx = ox + (col + 0.5) * res
            wy = oy + (row + 0.5) * res
            centroids.append((wx, wy))

        return centroids

    # ── Frontier selection ───────────────────────────────────────────────────

    def _select_best_frontier(
        self,
        centroids: list[tuple[float, float]],
        rx: float,
        ry: float,
    ) -> tuple[float, float] | None:
        """Return the nearest non-blacklisted centroid that is at least
        _MIN_GOAL_DIST_M from the robot after the inward shift, ensuring
        every goal movement is large enough to trigger a SLAM map update."""
        best_goal = None
        best_dist = float('inf')

        for cx, cy in centroids:
            if any(
                math.hypot(cx - bx, cy - by) < _BLACKLIST_RADIUS_M
                for bx, by in self._blacklist
            ):
                continue

            # Shift centroid 0.3 m toward robot so goal lands in clear free space
            dx, dy = rx - cx, ry - cy
            dist   = math.hypot(dx, dy)
            if dist > 0.3:
                gx = cx + 0.3 * (dx / dist)
                gy = cy + 0.3 * (dy / dist)
            else:
                gx, gy = cx, cy

            # Skip if the shifted goal is too close to trigger a map update
            goal_dist = math.hypot(gx - rx, gy - ry)
            if goal_dist < _MIN_GOAL_DIST_M:
                continue

            if goal_dist < best_dist:
                best_dist = goal_dist
                best_goal = (gx, gy)

        return best_goal

    # ── Goal sending ─────────────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'NavigateToPose action server not available; will retry.'
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # identity — planner handles heading

        goal   = NavigateToPose.Goal(pose=pose)
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

        self._navigating = True
        self._last_goal  = (x, y)
        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self._navigating = False
            if self._phase == _Phase.BOOTSTRAP:
                self._bootstrap_idx += 1  # skip rejected bootstrap leg
            else:
                self._abort_count += 1
            return
        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        # GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6

        # ── Bootstrap phase ──────────────────────────────────────────────────
        if self._phase == _Phase.BOOTSTRAP:
            if status in (4, 5, 6):
                self._bootstrap_idx += 1  # advance regardless of outcome
            self._navigating = False
            return

        # ── Explore phase ────────────────────────────────────────────────────
        if status == 4:
            self.get_logger().info('Reached frontier.')
            self._abort_count      = 0
            # Do NOT clear blacklist here — let it persist to avoid re-visiting
            # problematic frontiers. Blacklist is cleared only when all frontiers
            # are blacklisted (handled in _explore_once).
            self._map_settle_until = time.monotonic() + _MAP_SETTLE_S
        elif status == 5:
            self.get_logger().info('Goal canceled.')
        else:
            self.get_logger().warn(f'Navigation aborted (status={status}).')
            self._abort_count += 1
            if self._abort_count >= _ABORT_THRESHOLD and self._last_goal is not None:
                self.get_logger().warn(
                    f'Blacklisting frontier at {self._last_goal} after '
                    f'{_ABORT_THRESHOLD} aborts.'
                )
                self._blacklist.append(self._last_goal)
                self._abort_count = 0

        self._navigating = False

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {dist:.2f} m')

    # ── Main exploration loop ─────────────────────────────────────────────────

    def _explore_once(self):
        # ── Bootstrap phase ──────────────────────────────────────────────────
        if self._phase == _Phase.BOOTSTRAP:
            self._run_bootstrap()
            return

        # ── Explore phase ────────────────────────────────────────────────────
        if self._navigating:
            return

        # Wait for SLAM to incorporate new scans after reaching a frontier
        if time.monotonic() < self._map_settle_until:
            return

        if self._current_map is None:
            self.get_logger().warn('No map received yet.', throttle_duration_sec=5.0)
            return

        if self._check_stop_condition(self._current_map):
            self._explore_timer.cancel()
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        centroids = self._find_frontiers(self._current_map)
        if not centroids:
            self.get_logger().info(
                'No frontiers found — map may be fully explored. Stopping.'
            )
            self._explore_timer.cancel()
            return

        best = self._select_best_frontier(centroids, *robot_pose)
        if best is None:
            self.get_logger().info(
                'All reachable frontiers exhausted. Exploration complete. Stopping.'
            )
            self._explore_timer.cancel()
            return

        # Stuck detection: same frontier re-selected consecutively
        if self._last_goal is not None:
            if math.hypot(best[0] - self._last_goal[0],
                          best[1] - self._last_goal[1]) < _STUCK_GOAL_DIST_M:
                self._same_goal_count += 1
                if self._same_goal_count >= _SAME_GOAL_THRESHOLD:
                    self.get_logger().warn(
                        f'Same frontier re-selected {self._same_goal_count} times. '
                        f'Blacklisting {best}.'
                    )
                    self._blacklist.append(best)
                    self._same_goal_count = 0
                    return
            else:
                self._same_goal_count = 0
                self._abort_count = 0  # new frontier selected, reset abort counter

        self._send_goal(*best)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

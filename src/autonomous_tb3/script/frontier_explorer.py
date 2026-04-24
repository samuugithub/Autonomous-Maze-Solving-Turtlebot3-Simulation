#! /usr/bin/env python3

import math
import os
import subprocess
from collections import deque
from pathlib import Path

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return math.sin(half_yaw), math.cos(half_yaw)


class FrontierExplorer(BasicNavigator):
    def __init__(self):
        super().__init__()

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # In SLAM mode the map frame is anchored to the robot's first scan
        # position, which is always (0, 0) in map coordinates regardless of
        # where the robot was spawned in the Gazebo world.
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter(
            'map_save_prefix', os.path.expanduser('~/autonomous_tb3_maps/tb3_maze_map')
        )

        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.initial_pose_x = float(self.get_parameter('initial_pose_x').value)
        self.initial_pose_y = float(self.get_parameter('initial_pose_y').value)
        self.map_save_prefix = self.get_parameter('map_save_prefix').value

        self.map_msg = None
        self.next_report_percent = 5
        self.map_saved = False
        self.goal_active = False
        self.goal_history = deque(maxlen=12)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ReentrantCallbackGroup allows the timer to remain on the call stack
        # while the executor also processes action-client response callbacks
        # (goal acceptance, path result).  Without this, getPath() deadlocks
        # because the default MutuallyExclusiveCallbackGroup blocks action
        # responses while the timer callback is running.
        _cb_group = ReentrantCallbackGroup()
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 10,
                                  callback_group=_cb_group)
        self.create_timer(1.0, self._timer_callback, callback_group=_cb_group)

    def _map_callback(self, msg):
        self.map_msg = msg
        percent = self._known_percentage(msg)

        while percent >= self.next_report_percent:
            self.get_logger().info(
                f'[Mapping]: Exploration at {self.next_report_percent}% complete'
            )
            self.next_report_percent += 5

        if percent >= 95 and not self.map_saved:
            self._save_map()

    def _timer_callback(self):
        if not self._nav_stack_ready():
            return

        if self.map_msg is None:
            return

        if self.goal_active:
            if not self.isTaskComplete():
                return

            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Frontier goal completed successfully.')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Frontier goal was canceled.')
            elif result == TaskResult.FAILED:
                self.get_logger().warn('Frontier goal failed. Replanning.')
            else:
                self.get_logger().warn('Frontier goal returned an invalid status.')

            self.goal_active = False

        current_pose = self._current_robot_pose()
        if current_pose is None:
            return

        goal = self._select_frontier_goal(current_pose)
        if goal is None:
            if self._known_percentage(self.map_msg) >= 95 and not self.map_saved:
                self._save_map()
            return

        self.goal_history.append((goal.pose.position.x, goal.pose.position.y))
        self.get_logger().info(
            'Dispatching frontier goal: '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )
        self.goToPose(goal)
        self.goal_active = True

    def _nav_stack_ready(self):
        return self.nav_to_pose_client.wait_for_server(timeout_sec=0.0)

    def _known_percentage(self, msg):
        if not msg.data:
            return 0
        known_cells = sum(1 for cell in msg.data if cell != -1)
        return int(round((known_cells / len(msg.data)) * 100.0))

    def _current_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except TransformException:
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _cell_index(self, x, y, width):
        return y * width + x

    def _is_frontier_cell(self, x, y, data, width, height):
        index = self._cell_index(x, y, width)
        if data[index] != 0:
            return False

        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbor_value = data[self._cell_index(nx, ny, width)]
                if neighbor_value == -1:
                    return True
        return False

    def _frontier_clusters(self):
        msg = self.map_msg
        if msg is None:
            return []

        width = msg.info.width
        height = msg.info.height
        data = msg.data
        frontier_cells = set()

        for y in range(height):
            for x in range(width):
                if self._is_frontier_cell(x, y, data, width, height):
                    frontier_cells.add((x, y))

        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue

            queue = deque([cell])
            visited.add(cell)
            cluster = []
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    neighbor = (cx + dx, cy + dy)
                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            clusters.append(cluster)

        return clusters

    def _cluster_to_goal(self, cluster):
        msg = self.map_msg
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        sum_x = 0.0
        sum_y = 0.0
        for cell_x, cell_y in cluster:
            sum_x += origin_x + ((cell_x + 0.5) * resolution)
            sum_y += origin_y + ((cell_y + 0.5) * resolution)

        return sum_x / len(cluster), sum_y / len(cluster)

    def _world_in_map_bounds(self, x, y, margin=0.10):
        msg = self.map_msg
        if msg is None:
            return False

        min_x = msg.info.origin.position.x + margin
        min_y = msg.info.origin.position.y + margin
        max_x = msg.info.origin.position.x + (msg.info.width * msg.info.resolution) - margin
        max_y = msg.info.origin.position.y + (msg.info.height * msg.info.resolution) - margin
        return min_x <= x <= max_x and min_y <= y <= max_y

    def _pose_from_xy(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
        return pose

    def _select_frontier_goal(self, current_pose):
        clusters = self._frontier_clusters()
        if not clusters:
            return None

        robot_x, robot_y = current_pose
        candidates = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue

            goal_x, goal_y = self._cluster_to_goal(cluster)

            # Skip frontiers already attempted recently (avoids tight loops).
            if any(
                math.hypot(goal_x - hx, goal_y - hy) < 0.50
                for hx, hy in self.goal_history
            ):
                continue

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            candidates.append((distance, goal_x, goal_y))

        if not candidates:
            self.get_logger().debug(
                f'No frontier candidates (clusters={len(clusters)}, all filtered)'
            )
            return None

        # Pick the nearest frontier and send it directly.  Skipping getPath()
        # validation avoids blocking on a costmap that lags behind SLAM map
        # growth; Nav2 will report FAILED if the goal is truly unreachable.
        candidates.sort(key=lambda item: item[0])
        _, goal_x, goal_y = candidates[0]
        yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z, goal.pose.orientation.w = yaw_to_quaternion(yaw)
        return goal

    def _save_map(self):
        if self.map_saved:
            return

        prefix = Path(self.map_save_prefix).expanduser()
        prefix.parent.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Saving explored map to {prefix}')
        subprocess.Popen(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.map_saved = True


def main():
    rclpy.init()
    explorer = FrontierExplorer()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = explorer.get_clock().now().to_msg()
    initial_pose.pose.position.x = explorer.initial_pose_x
    initial_pose.pose.position.y = explorer.initial_pose_y
    initial_pose.pose.orientation.w = 1.0
    explorer.setInitialPose(initial_pose)

    try:
        explorer.get_logger().info('Frontier explorer is ready.')
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(explorer)
        executor.spin()
    finally:
        explorer.lifecycleShutdown()
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
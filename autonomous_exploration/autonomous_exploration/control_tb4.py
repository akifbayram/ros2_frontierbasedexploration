import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import math
import yaml
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import logging
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, CancelResponse
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion

from autonomous_exploration.timer import GoalTimer

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

with open("autonomous_exploration/config/params.yaml", 'r') as file:
    params = yaml.load(file, Loader=yaml.FullLoader)

expansion_size = params["expansion_size"]
target_error = params["target_error"]
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
frontierPointsGlobal = None

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def frontierB(matrix):
    height, width = matrix.shape
    for i in range(height):
        for j in range(width):
            if matrix[i][j] == 0.0:
                neighbors = [
                    (i - 1, j), (i + 1, j),
                    (i, j - 1), (i, j + 1),
                    (i - 2, j), (i + 2, j),
                    (i, j - 2), (i, j + 2)
                ]
                for ni, nj in neighbors:
                    if 0 <= ni < height and 0 <= nj < width:
                        if matrix[ni][nj] < 0:
                            matrix[i][j] = 2
                            break
    logger.debug("Frontier detection completed")
    return matrix

def dfs(matrix, i, j, visited, group_cells):
    stack = [(i, j)]
    height, width = matrix.shape
    while stack:
        ci, cj = stack.pop()
        if ci < 0 or ci >= height or cj < 0 or cj >= width:
            continue
        if matrix[ci][cj] != 2 or visited[ci][cj]:
            continue
        visited[ci][cj] = True
        group_cells.append((ci, cj))
        neighbors = [
            (ci + 1, cj), (ci - 1, cj),
            (ci, cj + 1), (ci, cj - 1),
            (ci + 2, cj), (ci - 2, cj),
            (ci, cj + 2), (ci, cj - 2)
        ]
        stack.extend(neighbors)

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    if n == 0:
        return None
    mean_x = sum(x_coords) / n
    mean_y = sum(y_coords) / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

def assign_groups(matrix):
    group_id = 1
    groups = []
    visited = np.zeros_like(matrix, dtype=bool)
    height, width = matrix.shape
    for i in range(height):
        for j in range(width):
            if matrix[i][j] == 2 and not visited[i][j]:
                group_cells = []
                dfs(matrix, i, j, visited, group_cells)
                if group_cells:
                    centroid = calculate_centroid([p[0] for p in group_cells], [p[1] for p in group_cells])
                    groups.append((group_id, group_cells, centroid))
                    group_id += 1
    logger.debug(f"Assigned {len(groups)} groups with centroids.")
    return matrix, groups

def fGroups(groups):
    sorted_groups = sorted(groups, key=lambda x: len(x[1]), reverse=True)
    top_groups = [g for g in sorted_groups if len(g[1]) > 2]
    logger.debug(f"Top groups selected: {[g[0] for g in top_groups]}")
    return top_groups

def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    data = data * resolution
    logger.debug("Costmap expansion completed")
    return data

def exploration(data, width, height, resolution, column, row, originX, originY):
    global frontierPointsGlobal
    data = costmap(data, width, height, resolution)
    if 0 <= row < height and 0 <= column < width:
        data[row][column] = 0
    else:
        logger.warning("Robot position is out of map bounds")
        return []
    data[data > 5] = 1
    data = frontierB(data)
    frontier_points = np.argwhere(data == 2)
    frontier_points_coords = []
    for point in frontier_points:
        r, c = point
        x = c * resolution + originX
        y = r * resolution + originY
        frontier_points_coords.append((x, y))
    frontierPointsGlobal = frontier_points_coords
    data, groups = assign_groups(data)
    groups = fGroups(groups)
    return groups

class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, best_effort_qos)
        logger.info("[INFO] EXPLORATION MODE ACTIVE")

        # Timers for regular updates
        self.timer_frontier = self.create_timer(0.5, self.publish_frontier_pointcloud)
        self.timer_centroid_markers = self.create_timer(0.5, self.publish_centroid_markers)
        self.timer_exploration = self.create_timer(1.0, self.exploration_loop)

        self.kesif = True
        self.start_time = None
        self.end_time = None
        self.total_exploration_time = None

        self.marker_publisher = self.create_publisher(Marker, 'frontier/marker', 10)
        self.goals_marker_publisher = self.create_publisher(MarkerArray, 'frontier/goals_markers', 10)
        self.goal_markers = []
        self.frontier_cloud_publisher = self.create_publisher(PointCloud2, 'frontier/frontier_points', 10)
        self.centroid_marker_publisher = self.create_publisher(MarkerArray, 'frontier/centroid_markers', 10)

        # Create an action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_to_pose_client.wait_for_server()

        # Initialize goal handle
        self.goal_handle = None
        self.current_goal = None

        # Define minimum distance threshold
        self.min_distance_threshold = 0.5  # Meters

        # Define goal tolerance
        self.goal_tolerance = 0.5  # Acceptable distance to consider goal reached

        # Initialize the GoalTimer
        self.goal_timer = GoalTimer(self, csv_filename='gdae_goal_times.csv')

    def exploration_loop(self):
        if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
            logger.debug("Waiting for map, odom, and scan data")
            return

        if self.start_time is None:
            self.start_time = time.perf_counter()
            # Start the GoalTimer when exploration starts
            self.goal_timer.start()
            logger.info("[INFO] Exploration started.")

        column = int((self.x - self.originX) / self.resolution)
        row = int((self.y - self.originY) / self.resolution)
        logger.debug(f"Current position: x={self.x}, y={self.y}, row={row}, column={column}")
        robot_position = (self.x, self.y)
        groups = exploration(
            self.data,
            self.width,
            self.height,
            self.resolution,
            column,
            row,
            self.originX,
            self.originY
        )
        self.groups = groups
        self.centroids = [g[2] for g in groups if g[2] is not None]  # Extract centroids for visualization

        if len(groups) == 0:
            if not self.kesif:
                logger.info("[INFO] EXPLORATION COMPLETED")
                self.end_time = time.perf_counter()
                self.total_exploration_time = self.end_time - self.start_time
                logger.info(f"[INFO] Total Exploration Time: {self.total_exploration_time:.2f} seconds")

                # Stop the GoalTimer and record the time in the CSV file
                elapsed_time = self.goal_timer.stop(success=True)
                if elapsed_time is not None:
                    logger.info(f"[INFO] Exploration time recorded: {elapsed_time:.2f} seconds")

                rclpy.shutdown()
            return

        # Select the best frontier based on distance and size
        best_centroid = self.select_best_frontier(groups, robot_position)
        if best_centroid is None:
            logger.info("[INFO] No suitable frontier found")
            return

        goal_x = best_centroid[1] * self.resolution + self.originX
        goal_y = best_centroid[0] * self.resolution + self.originY

        # If there is no current goal, or the new goal is better, update it
        if self.current_goal is None:
            if self.current_goal is not None:
                # Cancel current goal
                self.cancel_current_goal()
            self.send_goal(goal_x, goal_y)
            logger.info(f"[INFO] NEW TARGET ASSIGNED at ({goal_x}, {goal_y})")
            self.kesif = False
            self.current_goal = (goal_x, goal_y)
            self.create_goal_marker(goal_x, goal_y)
            self.publish_goal_markers()
        else:
            logger.debug("[INFO] Continuing with current goal")

    def cancel_current_goal(self):
        if hasattr(self, 'goal_handle_future') and self.goal_handle_future is not None and not self.goal_handle_future.done():
            self.goal_handle_future.cancel()
        elif self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            logger.info("[INFO] Current goal cancelled")

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response == CancelResponse.ACCEPT:
            logger.info("[INFO] Goal cancellation accepted")
        else:
            logger.info("[INFO] Goal cancellation rejected")
        self.goal_handle = None

    def select_best_frontier(self, groups, robot_position):
        best_score = -float('inf')
        best_centroid = None
        robot_x, robot_y = robot_position

        for group_id, group_cells, centroid in groups:
            if centroid is None:
                continue
            group_size = len(group_cells)

            centroid_x = centroid[1] * self.resolution + self.originX
            centroid_y = centroid[0] * self.resolution + self.originY

            distance = math.hypot(centroid_x - robot_x, centroid_y - robot_y)

            # Skip frontiers that are too close
            if distance < self.min_distance_threshold:
                logger.debug(f"Frontier {group_id} is too close ({distance:.2f} m), skipping.")
                continue

            # Calculate score (you can adjust the weightings as needed)
            score = group_size / (distance + 1e-6)  # Add small epsilon to avoid division by zero

            logger.debug(f"Frontier {group_id}: size={group_size}, distance={distance:.2f}, score={score:.4f}")

            if score > best_score:
                best_score = score
                best_centroid = centroid

        return best_centroid

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation to face forward (optional)
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
        goal_msg.pose.pose.orientation = quaternion

        # Send goal and register callbacks
        self.goal_handle_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.goal_handle_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.info('Goal rejected by Nav2')
                self.kesif = True  # Try another goal
                self.current_goal = None
                return

            logger.info('Goal accepted by Nav2')
            self.goal_handle = goal_handle
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            logger.error(f'Exception in goal_response_callback: {e}')
            self.kesif = True
            self.current_goal = None

    def get_result_callback(self, future):
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                logger.info('Goal succeeded!')
            else:
                logger.info(f'Goal failed with status: {status}')
            self.kesif = True  # Proceed to the next goal
            self.current_goal = None
        except Exception as e:
            logger.error(f'Exception in get_result_callback: {e}')
            self.kesif = True
            self.current_goal = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = time.perf_counter()

        # Calculate distance to goal
        if self.current_goal is not None:
            distance_to_goal = math.hypot(self.current_goal[0] - self.x, self.current_goal[1] - self.y)
            logger.debug(f'Distance to goal: {distance_to_goal:.2f} m')
            if distance_to_goal <= self.goal_tolerance:
                logger.info(f'Goal reached within tolerance of {self.goal_tolerance} m.')
                # Cancel the goal
                self.cancel_current_goal()
                self.kesif = True  # Proceed to the next goal
                self.current_goal = None

    def scan_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges
        logger.debug("Scan data received")

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data
        logger.debug("Map data received")

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        logger.debug(f"Odom data received: x={self.x}, y={self.y}, yaw={self.yaw}")

    def publish_frontier_pointcloud(self):
        global frontierPointsGlobal

        if frontierPointsGlobal is None or len(frontierPointsGlobal) == 0:
            empty_cloud = pcl2.create_cloud_xyz32(Header(frame_id='map', stamp=self.get_clock().now().to_msg()), [])
            self.frontier_cloud_publisher.publish(empty_cloud)
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        points = [(x, y, 0.0) for x, y in frontierPointsGlobal]
        cloud = pcl2.create_cloud_xyz32(header, points)
        self.frontier_cloud_publisher.publish(cloud)

    def publish_centroid_markers(self):
        if hasattr(self, 'centroids') and self.centroids:
            centroid_marker_array = MarkerArray()

            delete_marker = Marker()
            delete_marker.header.frame_id = 'map'
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = 'frontier_centroids'
            delete_marker.action = Marker.DELETEALL
            centroid_marker_array.markers.append(delete_marker)

            for idx, centroid in enumerate(self.centroids):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'frontier_centroids'
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Transform centroid from grid coordinates to map coordinates
                marker.pose.position.x = centroid[1] * self.resolution + self.originX
                marker.pose.position.y = centroid[0] * self.resolution + self.originY
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                centroid_marker_array.markers.append(marker)

            self.centroid_marker_publisher.publish(centroid_marker_array)

    def create_goal_marker(self, goal_x, goal_y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goals_history'
        marker.id = len(self.goal_markers)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.goal_markers.append(marker)

    def publish_goal_markers(self):
        marker_array = MarkerArray()
        marker_array.markers = self.goal_markers
        self.goals_marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

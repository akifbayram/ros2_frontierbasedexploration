import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import heapq , math , random , yaml
import scipy.interpolate as si
import sys , threading , time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from autonomous_exploration.timer import GoalTimer

with open("autonomous_exploration/config/params.yaml", 'r') as file:
    params = yaml.load(file, Loader=yaml.FullLoader)

lookahead_distance = params["lookahead_distance"]
speed = params["speed"]
expansion_size = params["expansion_size"]
target_error = params["target_error"]
robot_r = params["robot_r"]
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,  
    depth=10  
)
pathGlobal = 0
frontierPointsGlobal = None

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
                elif i > 1 and matrix[i-2][j] < 0: 
                    matrix[i][j] = 2
                elif i < len(matrix)-2 and matrix[i+2][j] < 0:
                    matrix[i][j] = 2
                elif j > 1 and matrix[i][j-2] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-2 and matrix[i][j+2] < 0:
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group = 1
    groups = {}
    centroids = []
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    for group_id, points in groups.items():
        if len(points) > 0:
            centroid = calculate_centroid([p[0] for p in points], [p[1] for p in points])
            centroids.append(centroid)
    return matrix, groups, centroids

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0 
    
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 2, j, group, groups) 
    dfs(matrix, i - 2, j, group, groups)
    dfs(matrix, i, j + 2, group, groups)
    dfs(matrix, i, j - 2, group, groups)

    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

# Bu fonksiyon en buyuk 5 gruptan target_error*2 uzaklıktan daha uzak olan ve robota en yakın olanı seçer.
"""
def findClosestGroup(matrix,groups, current,resolution,originX,originY):
    targetP = None
    distances = []
    paths = []
    min_index = -1
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        path = astar(matrix, current, middle)
        path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
        total_distance = pathLength(path)
        distances.append(total_distance)
        paths.append(path)
    for i in range(len(distances)):
        if distances[i] > target_error*3:
            if min_index == -1 or distances[i] < distances[min_index]:
                min_index = i
    if min_index != -1:
        targetP = paths[min_index]
    else: #gruplar target_error*2 uzaklıktan daha yakınsa random bir noktayı hedef olarak seçer. Bu robotun bazı durumlardan kurtulmasını sağlar.
        index = random.randint(0,len(groups)-1)
        target = groups[index][1]
        target = target[random.randint(0,len(target)-1)]
        path = astar(matrix, current, target)
        targetP = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
    return targetP
"""
def findClosestGroup(matrix,groups, current,resolution,originX,originY):
    targetP = None
    distances = []
    paths = []
    score = []
    max_score = -1 # max score index
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        path = astar(matrix, current, middle)
        path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
        total_distance = pathLength(path)
        distances.append(total_distance)
        paths.append(path)
    for i in range(len(distances)):
        if distances[i] == 0:
            score.append(0)
        else:
            score.append(len(groups[i][1]) / math.exp(distances[i]))  # Strongly penalizes larger distances
    for i in range(len(distances)):
        if distances[i] > target_error*3:
            if max_score == -1 or score[i] > score[max_score]:
                max_score = i
    if max_score != -1:
        targetP = paths[max_score]
    else: # gruplar target_error*2 uzaklıktan daha yakınsa random bir noktayı hedef olarak seçer. Bu robotun bazı durumlardan kurtulmasını sağlar.
        index = random.randint(0,len(groups)-1)
        target = groups[index][1]
        target = target[random.randint(0,len(target)-1)]
        path = astar(matrix, current, target)
        targetP = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
    return targetP

def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def exploration(data,width,height,resolution,column,row,originX,originY):
        global pathGlobal, frontierPointsGlobal # Global degisken
        data = costmap(data,width,height,resolution) #Engelleri genislet
        data[row][column] = 0 # Robot Anlık Konum
        data[data > 5] = 1 # 0 olanlar gidilebilir yer, 100 olanlar kesin engel
        data = frontierB(data) # Sınır noktaları bul

        frontier_points = np.argwhere(data == 2)
        frontier_points_coords = []
        for point in frontier_points:
            r, c = point
            x = c * resolution + originX
            y = r * resolution + originY
            frontier_points_coords.append((x, y))
        frontierPointsGlobal = frontier_points_coords 


        data,groups,centroids = assign_groups(data) #Sınır noktaları gruplandır
        groups = fGroups(groups) # Grupları küçükten büyüğe sırala. En buyuk 5 grubu al
        if len(groups) == 0: # Grup yoksa kesif tamamlandı
            path = -1
        else: # Grup varsa en yakın grubu bul
            data[data < 0] = 1 #-0.05 olanlar bilinmeyen yer. Gidilemez olarak isaretle. 0 = gidilebilir, 1 = gidilemez.
            path = findClosestGroup(data,groups,(row,column),resolution,originX,originY) #En yakın grubu bul
            if path != None: # Yol varsa BSpline ile düzelt
                path = bspline_planning(path,len(path)*5)
            else:
                path = -1
        pathGlobal = path
        return centroids

def localControl(scan):
    v = None
    w = None
    for i in range(60):
        if scan[i] < robot_r:
            v = 0.2
            w = -math.pi/4 
            break
    if v == None:
        for i in range(300,360):
            if scan[i] < robot_r:
                v = 0.2
                w = math.pi/4
                break
    return v,w


class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription_map = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, best_effort_qos)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        print("[INFO] EXPLORATION MODE ACTIVE")

        self.timer_frontier = self.create_timer(0.1, self.publish_frontier_pointcloud)
        self.timer_exploration = self.create_timer(0.1, self.exploration_loop)
        self.timer_centroid_markers = self.create_timer(0.1, self.publish_centroid_markers)

        self.kesif = True
        self.path = None
        self.i = 0
        self.start_time = None
        self.end_time = None  
        self.total_exploration_time = None

        self.path_publisher = self.create_publisher(Path, 'plan', 10)
        self.marker_publisher = self.create_publisher(Marker, 'marker', 10)
        self.robot_path_publisher = self.create_publisher(Path, 'robot_path', 10)
        self.robot_path = []
        self.goals_marker_publisher = self.create_publisher(MarkerArray, 'goals_markers', 10)
        self.goal_markers = []
        self.frontier_cloud_publisher = self.create_publisher(PointCloud2, 'frontier_points', 10)
        self.centroid_marker_publisher = self.create_publisher(MarkerArray, 'centroid_markers', 10)

        # Initialize the GoalTimer
        self.goal_timer = GoalTimer(self, csv_filename='fbae_goal_times.csv')

    def exploration_loop(self):
        twist = Twist()

        if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
            return

        if self.kesif:
            if self.start_time is None:
                self.start_time = time.perf_counter()
                self.goal_timer.start()  # Start the timer
                print("[INFO] Exploration started.")
                
            column = int((self.x - self.originX) / self.resolution)
            row = int((self.y - self.originY) / self.resolution)
            centroids = exploration(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)
            self.centroids = centroids  # Store centroids in the navigationControl class
            self.path = pathGlobal

            if isinstance(self.path, int) and self.path == -1:
                print("[INFO] EXPLORATION COMPLETED")
                self.end_time = time.perf_counter()
                self.total_exploration_time = self.end_time - self.start_time
                print(f"[INFO] Total Exploration Time: {self.total_exploration_time:.2f} seconds")

                # Stop the GoalTimer and record the time
                elapsed_time = self.goal_timer.stop(success=True)
                if elapsed_time is not None:
                    print(f"[INFO] Exploration time recorded: {elapsed_time:.2f} seconds")

                rclpy.shutdown()
            else:
                goal_x = self.path[-1][0]
                goal_y = self.path[-1][1]
                self.publish_goal_marker(goal_x, goal_y)
                self.create_goal_marker(goal_x, goal_y)
                self.publish_goal_markers()
                self.publish_path(self.path)
                self.i = 0
                print("[INFO] NEW TARGET ASSIGNED")
            self.kesif = False
        else:
            v, w = localControl(self.scan)
            if v is None:
                v, w, self.i = pure_pursuit(self.x, self.y, self.yaw, self.path, self.i)
            if abs(self.x - self.path[-1][0]) < target_error and abs(self.y - self.path[-1][1]) < target_error:
                v = 0.0
                w = 0.0
                self.kesif = True
                print("[INFO] TARGET REACHED")
            twist.linear.x = v
            twist.angular.z = w
            self.publisher.publish(twist)
            
    def scan_callback(self,msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

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

        self.robot_path.append((self.x, self.y))
        self.publish_robot_path()

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            poses.append(pose)
        path_msg.poses = poses
        self.path_publisher.publish(path_msg)

    def publish_goal_marker(self, goal_x, goal_y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
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
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

    def publish_robot_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for p in self.robot_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            poses.append(pose)
        path_msg.poses = poses
        self.robot_path_publisher.publish(path_msg)

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
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                centroid_marker_array.markers.append(marker)

            self.centroid_marker_publisher.publish(centroid_marker_array)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
import math
import numpy as np
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

# Global variables
goal_x = None
goal_y = None
odom_info = None
scan_info = None
goal_done = False


# Take goal coordinates from user
def set_goal():
    global goal_x, goal_y, goal_done
    goal_x = float(input("Enter goal X: "))
    goal_y = float(input("Enter goal Y: "))
    goal_done = False


# Odometry callback
def odom_callback(msg):
    global odom_info
    odom_info = msg


# Laser scan callback
def scan_callback(msg):
    global scan_info
    scan_info = msg


# Predict how robot will move for given speed and turn
def simulate_motion(linear_v, angular_v, dt):
    if odom_info is None:
        return []

    x = odom_info.pose.pose.position.x
    y = odom_info.pose.pose.position.y
    orientation = odom_info.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    predicted_path = []
    for _ in range(100):
        yaw += angular_v * dt
        x += linear_v * math.cos(yaw) * dt
        y += linear_v * math.sin(yaw) * dt
        predicted_path.append((x, y))

    return predicted_path


# Check if path hits any obstacle
def collision_check(path_points):
    if scan_info is None:
        return -float('inf')

    safe_gap = 0.3
    for x, y in path_points:
        dist = math.sqrt(x ** 2 + y ** 2)
        index = int(math.atan2(y, x) * (len(scan_info.ranges) / (2 * math.pi)))
        index = max(0, min(len(scan_info.ranges) - 1, index))

        if dist < scan_info.ranges[index] - safe_gap:
            return -100000
    return 0


# Pick best motion from all options
def pick_best_path(node, paths):
    global goal_x, goal_y, goal_done

    if odom_info is None:
        return 0.0, 0.0

    curr_x = odom_info.pose.pose.position.x
    curr_y = odom_info.pose.pose.position.y
    orientation = odom_info.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    dist_to_goal = math.hypot(goal_x - curr_x, goal_y - curr_y)
    if dist_to_goal < 0.05:
        if not goal_done:
            goal_done = True
            node.get_logger().info(f"Goal reached at ({goal_x}, {goal_y})")
        return 0.0, 0.0

    best_score = float('-inf')
    best_v, best_w = 0.05, 0.0

    for v, w, path in paths:
        goal_dist_score = -math.hypot(path[-1][0] - goal_x, path[-1][1] - goal_y) * 5
        angle_to_goal = math.atan2(goal_y - curr_y, goal_x - curr_x)
        heading_score = -abs(angle_to_goal - yaw) * 2
        collision_score = collision_check(path)
        smooth_score = -0.1 * abs(w)

        total = goal_dist_score + heading_score + collision_score + smooth_score

        if total > best_score:
            best_score = total
            best_v, best_w = v, w

    return best_v, best_w


# Generate random paths continuously
def make_paths(max_v, max_w, dt):
    while True:
        v = random.uniform(0, max_v)
        w = random.uniform(-max_w, max_w)
        yield (v, w, simulate_motion(v, w, dt))


# Main movement logic
def control_loop(node, cmd_pub, path_pub, max_v, max_w, dt):
    global goal_done

    if odom_info is None or scan_info is None or goal_done:
        return

    gen = make_paths(max_v, max_w, dt)
    options = [next(gen) for _ in range(10000)]

    v, w = pick_best_path(node, options)

    move = Twist()
    move.linear.x = v
    move.angular.z = w
    cmd_pub.publish(move)

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.color.r = 1.0
    marker.color.a = 0.5

    for _, _, path in options:
        for x, y in path:
            p = Point()
            p.x = x
            p.y = y
            marker.points.append(p)

    path_pub.publish(marker)


def main():
    rclpy.init()
    node = Node('dwa_planner')

    set_goal()

    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    path_pub = node.create_publisher(Marker, '/visual_paths', 10)

    max_v = 0.15
    max_w = 2.5
    dt = 0.1

    node.create_timer(dt, lambda: control_loop(node, cmd_pub, path_pub, max_v, max_w, dt))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

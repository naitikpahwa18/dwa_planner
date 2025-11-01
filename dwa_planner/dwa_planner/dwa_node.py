#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Header

# PARAMETERS (tweak these to adapt robot / environment)
MAX_LINEAR_V = 0.22        # m/s (turtlebot3 default ~0.22)
MIN_LINEAR_V = 0.0
MAX_ANGULAR_V = 2.84       # rad/s (turtlebot3 default ~2.84)
MAX_LINEAR_ACC = 0.5       # m/s^2 (how quickly linear vel can change)
MAX_ANGULAR_ACC = 3.0      # rad/s^2
SIM_TIME = 1.2             # seconds to simulate each trajectory
SIM_DT = 0.1               # integration timestep
NUM_SAMPLES_V = 7
NUM_SAMPLES_W = 21
SAFETY_DISTANCE = 0.20     # meters (distance to obstacle considered unsafe)
GOAL_TOLERANCE = 0.10      # meters
ANGLE_TOLERANCE = 0.2      # radians

# Cost weights
WEIGHT_GOAL = 5.0
WEIGHT_HEADING = 2.0
WEIGHT_CLEARANCE = 6.0
WEIGHT_SPEED = 1.0
WEIGHT_SMOOTH = 0.5


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # internal state
        self.odom = None
        self.scan = None
        self.goal = None
        self.goal_reached = False
        self.prev_cmd = Twist()

        # subscriptions / publishers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visual_paths', 10)

        # Timer for control loop
        control_dt = SIM_DT
        self.create_timer(control_dt, self.control_loop)

        self.get_logger().info('DWA Planner node initialized.')

        # ask user for goal (blocking once at startup)
        self.ask_for_goal()

    def ask_for_goal(self):
        try:
            gx = float(input("Enter goal X: "))
            gy = float(input("Enter goal Y: "))
        except Exception as e:
            self.get_logger().warn('Invalid goal input, defaulting to (2.0, 1.0)')
            gx, gy = 2.0, 1.0
        self.goal = (gx, gy)
        self.goal_reached = False
        self.get_logger().info(f'New goal: ({gx:.2f}, {gy:.2f})')

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    @staticmethod
    def normalize_angle(angle):
        a = (angle + math.pi) % (2.0 * math.pi) - math.pi
        return a

    def dynamic_window(self, current_v, current_w, dt=SIM_DT):
        # compute reachable velocity window given acceleration limits and robot limits
        min_v = max(MIN_LINEAR_V, current_v - MAX_LINEAR_ACC * dt)
        max_v = min(MAX_LINEAR_V, current_v + MAX_LINEAR_ACC * dt)
        min_w = max(-MAX_ANGULAR_V, current_w - MAX_ANGULAR_ACC * dt)
        max_w = min(MAX_ANGULAR_V, current_w + MAX_ANGULAR_ACC * dt)
        return (min_v, max_v, min_w, max_w)

    def simulate_trajectory(self, x, y, yaw, v, w, sim_time=SIM_TIME, dt=SIM_DT):
        # simulate forward using simple unicycle model (global coordinates)
        path = []
        t = 0.0
        cx, cy, cyaw = x, y, yaw
        while t < sim_time:
            # integrate
            cx += v * math.cos(cyaw) * dt
            cy += v * math.sin(cyaw) * dt
            cyaw += w * dt
            path.append((cx, cy, cyaw))
            t += dt
        return path

    def evaluate_trajectory(self, path, final_v, final_w):
        # compute costs: distance-to-goal (final point), heading, clearance (min dist to obstacles), speed preference, smoothness
        if self.goal is None:
            return -float('inf')

        gx, gy = self.goal
        fx, fy, fyaw = path[-1]

        # goal distance cost (lower distance -> higher score)
        dist_to_goal = math.hypot(gx - fx, gy - fy)
        goal_cost = -WEIGHT_GOAL * dist_to_goal

        # heading cost: difference between final orientation and heading to goal
        heading_to_goal = math.atan2(gy - fy, gx - fx)
        ang_diff = abs(self.normalize_angle(heading_to_goal - fyaw))
        heading_cost = -WEIGHT_HEADING * ang_diff

        # clearance: compute min distance from path points to obstacles using LaserScan
        clearance = self.compute_min_range_along_path(path)
        # if any point inside unsafe margin => big negative penalty
        if clearance is None:
            # no scan available -> be conservative but not fatal
            clearance_cost = -WEIGHT_CLEARANCE * 0.5
        else:
            if clearance < SAFETY_DISTANCE:
                # big negative
                return -1e6
            clearance_cost = WEIGHT_CLEARANCE * clearance

        # speed preference (prefer faster forward speed)
        speed_cost = WEIGHT_SPEED * final_v

        # smoothness / angular penalty (prefer small angular velocities)
        smooth_cost = -WEIGHT_SMOOTH * abs(final_w)

        total = goal_cost + heading_cost + clearance_cost + speed_cost + smooth_cost
        return total

    def compute_min_range_along_path(self, path):
        # returns min range to obstacles along path in meters
        if self.scan is None:
            return None

        ranges = self.scan.ranges
        angle_min = self.scan.angle_min
        angle_inc = self.scan.angle_increment
        n = len(ranges)

        # protective conversion: some ranges may be 0 or inf
        safe_min = float('inf')
        # need robot pose to compute relative angle: use current odom
        if self.odom is None:
            return None
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        orient = self.odom.pose.pose.orientation
        _, _, ryaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        for (px, py, _) in path:
            dx = px - rx
            dy = py - ry
            dist = math.hypot(dx, dy)
            ang = math.atan2(dy, dx) - ryaw
            ang = self.normalize_angle(ang)
            idx = int((ang - angle_min) / angle_inc)
            if idx < 0 or idx >= n:
                # point is outside scan field-of-view -> treat as large distance
                continue
            r = ranges[idx]
            # some scan returns nan/inf/0; guard against that
            try:
                if math.isfinite(r) and r > 0.001:
                    safe_min = min(safe_min, r - dist)  # r - dist is clearance left at that index
                else:
                    # unreliable measurement - skip
                    continue
            except Exception:
                continue

        if safe_min == float('inf'):
            # no reliable info
            return None
        return safe_min

    def control_loop(self):
        # main DWA loop
        if self.odom is None:
            # nothing to do yet
            return
        if self.goal is None:
            return
        if self.goal_reached:
            # still publish zero cmd to ensure robot stops
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        # current pose and velocities
        ox = self.odom.pose.pose.position.x
        oy = self.odom.pose.pose.position.y
        orient = self.odom.pose.pose.orientation
        _, _, oyaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        current_v = 0.0
        current_w = 0.0
        try:
            current_v = self.odom.twist.twist.linear.x
            current_w = self.odom.twist.twist.angular.z
        except Exception:
            current_v = 0.0
            current_w = 0.0

        # check goal reached
        gx, gy = self.goal
        dist_goal = math.hypot(gx - ox, gy - oy)
        if dist_goal < GOAL_TOLERANCE:
            # also check orientation optionally
            self.get_logger().info(f"Goal reached: distance {dist_goal:.3f}m.")
            self.goal_reached = True
            # stop robot
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        # dynamic window
        min_v, max_v, min_w, max_w = self.dynamic_window(current_v, current_w, dt=SIM_DT)

        # structured sampling
        v_samples = np.linspace(min_v, max_v, NUM_SAMPLES_V)
        w_samples = np.linspace(min_w, max_w, NUM_SAMPLES_W)

        best_score = -float('inf')
        best_v = 0.0
        best_w = 0.0
        best_path = None

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = 'map'  # publish in map frame (poses are in odom/map depending on tf; odom positions are in odom)
        marker.ns = 'dwa_paths'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.a = 0.6
        marker.pose.orientation.w = 1.0
        marker.points = []

        # evaluate each sampled trajectory
        for v in v_samples:
            for w in w_samples:
                path = self.simulate_trajectory(ox, oy, oyaw, float(v), float(w), sim_time=SIM_TIME, dt=SIM_DT)
                score = self.evaluate_trajectory(path, float(v), float(w))
                # append visualization (only a subset for performance)
                if score > best_score:
                    best_score = score
                    best_v = float(v)
                    best_w = float(w)
                    best_path = path

                # add path lines to marker (pairs of points)
                # do not add too many points to RViz; sample some
                if len(path) > 1 and (abs(v - best_v) < 1e-6 or score > best_score - 0.001):
                    for i in range(0, len(path)-1):
                        p1 = Point()
                        p2 = Point()
                        p1.x, p1.y = path[i][0], path[i][1]
                        p2.x, p2.y = path[i+1][0], path[i+1][1]
                        marker.points.append(p1)
                        marker.points.append(p2)

        # if no feasible path found, attempt an in-place rotation search
        if best_path is None:
            self.get_logger().warn('No feasible path: stopping and attempting in-place rotation to clear obstacles.')
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # small rotation
            self.cmd_pub.publish(cmd)
            return

        # apply acceleration limits smoothly to the chosen commands
        dt = SIM_DT
        cmd = Twist()
        # linear
        dv_max = MAX_LINEAR_ACC * dt
        desired_v = best_v
        prev_v = self.prev_cmd.linear.x
        v_diff = desired_v - prev_v
        if v_diff > dv_max:
            cmd.linear.x = prev_v + dv_max
        elif v_diff < -dv_max:
            cmd.linear.x = prev_v - dv_max
        else:
            cmd.linear.x = desired_v

        # angular
        dw_max = MAX_ANGULAR_ACC * dt
        desired_w = best_w
        prev_w = self.prev_cmd.angular.z
        w_diff = desired_w - prev_w
        if w_diff > dw_max:
            cmd.angular.z = prev_w + dw_max
        elif w_diff < -dw_max:
            cmd.angular.z = prev_w - dw_max
        else:
            cmd.angular.z = desired_w

        # final safety clamp
        cmd.linear.x = float(max(MIN_LINEAR_V, min(cmd.linear.x, MAX_LINEAR_V)))
        cmd.angular.z = float(max(-MAX_ANGULAR_V, min(cmd.angular.z, MAX_ANGULAR_V)))

        self.cmd_pub.publish(cmd)
        self.prev_cmd = cmd

        # publish marker
        try:
            self.marker_pub.publish(marker)
        except Exception:
            pass

        # debug logging occasionally
        if self.get_clock().now().nanoseconds % (1_000_000_000 * 1) < 1:  # rough periodic print
            self.get_logger().debug(
                f"Chosen v={cmd.linear.x:.3f} w={cmd.angular.z:.3f} score={best_score:.1f} dist_goal={dist_goal:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DWA planner')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


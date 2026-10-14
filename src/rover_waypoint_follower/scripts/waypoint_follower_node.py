#!/usr/bin/env python3
import csv
import math
import os

from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class WaypointFollower(Node):
    """Twist-based waypoint follower with real-time RViz TF and Marker visualization."""

    def __init__(self):
        super().__init__('waypoint_follower')

        # --- Parameters -------------------------------------------------
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('boundary_csv', '')

        waypoints_path = self.get_parameter('waypoints_csv').get_parameter_value().string_value
        boundary_path = self.get_parameter('boundary_csv').get_parameter_value().string_value

        self.waypoints = self._read_waypoints(waypoints_path)
        self.boundary = self._read_boundary(boundary_path)

        self.idx = 0
        self.is_completed = False
        self.get_logger().info('Waypoints loaded: %d' % len(self.waypoints))
        self.get_logger().info('Boundary: %s' % self.boundary)

        # Pose state
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_th = 0.0
        self.has_odom = False
        self.last_cmd = Twist()
        self.last_step_time = self.get_clock().now()

        # --- TF & Visualization -----------------------------------------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.path_pub = self.create_publisher(Path, '/rover_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        self.rover_path = Path()
        self.rover_path.header.frame_id = 'odom'

        # --- Subscriptions ----------------------------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # --- Publishers -------------------------------------------------
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        # --- Timer -------------------------------------------------------
        self.timer = self.create_timer(0.1, self.step)  # 10 Hz

    def _odom_callback(self, msg: Odometry):
        self.has_odom = True
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_th = math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _read_waypoints(path: str):
        rows = []
        if not os.path.isfile(path):
            raise FileNotFoundError('Waypoints file not found: %s' % path)
        with open(path, newline='') as f:
            reader = csv.reader(f)
            for r in reader:
                if not r:
                    continue
                first = r[0].strip()
                if not first or first.startswith('#'):
                    continue
                x = float(first)
                y = float(r[1].strip())
                theta = float(r[2].strip()) if len(r) > 2 else 0.0
                rows.append((x, y, theta))
        return rows

    @staticmethod
    def _read_boundary(path: str):
        if not os.path.isfile(path):
            raise FileNotFoundError('Boundary file not found: %s' % path)
        with open(path, newline='') as f:
            reader = csv.reader(f)
            for r in reader:
                if not r:
                    continue
                first = r[0].strip()
                if not first or first.startswith('#'):
                    continue
                vals = [float(v.strip()) for v in r]
                if len(vals) != 4:
                    raise ValueError('Boundary CSV must have exactly 4 values (x_min,x_max,y_min,y_max)')
                return {
                    'x_min': vals[0],
                    'x_max': vals[1],
                    'y_min': vals[2],
                    'y_max': vals[3],
                }
        raise ValueError('No valid boundary data found in %s' % path)

    @staticmethod
    def _inside_boundary(x: float, y: float, b: dict) -> bool:
        return (b['x_min'] <= x <= b['x_max'] and
                b['y_min'] <= y <= b['y_max'])

    def _broadcast_tf(self, now):
        # Broadcast odom -> base_footprint transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(self.pose_x)
        t.transform.translation.y = float(self.pose_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.pose_th / 2.0)
        t.transform.rotation.w = math.cos(self.pose_th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publish rover path trail
        pose_stamped = PoseStamped()
        pose_stamped.header = t.header
        pose_stamped.pose.position.x = float(self.pose_x)
        pose_stamped.pose.position.y = float(self.pose_y)
        pose_stamped.pose.position.z = 0.02
        pose_stamped.pose.orientation = t.transform.rotation

        # Only append if moved significantly to avoid path bloating
        if not self.rover_path.poses or math.hypot(
            self.rover_path.poses[-1].pose.position.x - self.pose_x,
            self.rover_path.poses[-1].pose.position.y - self.pose_y
        ) > 0.02:
            self.rover_path.poses.append(pose_stamped)
            self.rover_path.header.stamp = now.to_msg()
            self.path_pub.publish(self.rover_path)

    def _publish_markers(self, now):
        marker_array = MarkerArray()

        # 1. Boundary Box Line Strip
        b_marker = Marker()
        b_marker.header.frame_id = 'odom'
        b_marker.header.stamp = now.to_msg()
        b_marker.ns = 'boundary'
        b_marker.id = 0
        b_marker.type = Marker.LINE_STRIP
        b_marker.action = Marker.ADD
        b_marker.scale.x = 0.03  # Line width
        b_marker.color.r = 0.2
        b_marker.color.g = 0.8
        b_marker.color.b = 1.0
        b_marker.color.a = 0.9

        x0, x1 = self.boundary['x_min'], self.boundary['x_max']
        y0, y1 = self.boundary['y_min'], self.boundary['y_max']
        for px, py in [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]:
            p = Point()
            p.x, p.y, p.z = float(px), float(py), 0.0
            b_marker.points.append(p)
        marker_array.markers.append(b_marker)

        # 2. Waypoint Spheres & Dynamic Tracking
        for i, (wx, wy, _) in enumerate(self.waypoints):
            inside = self._inside_boundary(wx, wy, self.boundary)
            wp_marker = Marker()
            wp_marker.header.frame_id = 'odom'
            wp_marker.header.stamp = now.to_msg()
            wp_marker.ns = 'waypoints'
            wp_marker.id = i + 10
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = float(wx)
            wp_marker.pose.position.y = float(wy)
            wp_marker.pose.position.z = 0.05

            if not inside:
                wp_marker.scale.x = 0.08
                wp_marker.scale.y = 0.08
                wp_marker.scale.z = 0.08
                wp_marker.color.r = 0.9
                wp_marker.color.g = 0.2
                wp_marker.color.b = 0.2
                wp_marker.color.a = 0.4
            elif i < self.idx:
                # Visited: Gold
                wp_marker.scale.x = 0.09
                wp_marker.scale.y = 0.09
                wp_marker.scale.z = 0.09
                wp_marker.color.r = 1.0
                wp_marker.color.g = 0.8
                wp_marker.color.b = 0.1
                wp_marker.color.a = 0.75
            elif i == self.idx:
                # Current Target: Cyan / Bright Blue
                wp_marker.scale.x = 0.18
                wp_marker.scale.y = 0.18
                wp_marker.scale.z = 0.18
                wp_marker.color.r = 0.0
                wp_marker.color.g = 0.9
                wp_marker.color.b = 1.0
                wp_marker.color.a = 1.0
            else:
                # Upcoming: Green
                wp_marker.scale.x = 0.10
                wp_marker.scale.y = 0.10
                wp_marker.scale.z = 0.10
                wp_marker.color.r = 0.1
                wp_marker.color.g = 0.9
                wp_marker.color.b = 0.2
                wp_marker.color.a = 0.85
            marker_array.markers.append(wp_marker)

        # 3. Dynamic Status Label above Target
        status_marker = Marker()
        status_marker.header.frame_id = 'odom'
        status_marker.header.stamp = now.to_msg()
        status_marker.ns = 'status'
        status_marker.id = 999
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        if self.idx < len(self.waypoints):
            twx, twy, _ = self.waypoints[self.idx]
            status_marker.pose.position.x = float(twx)
            status_marker.pose.position.y = float(twy)
            status_marker.pose.position.z = 0.30
            status_marker.scale.z = 0.14
            status_marker.color.r = 0.0
            status_marker.color.g = 1.0
            status_marker.color.b = 1.0
            status_marker.color.a = 1.0
            status_marker.text = f">> Target: WP {self.idx}/50 ({twx:.1f}, {twy:.1f}) <<"
        else:
            status_marker.pose.position.x = 0.0
            status_marker.pose.position.y = 0.0
            status_marker.pose.position.z = 0.40
            status_marker.scale.z = 0.20
            status_marker.color.r = 0.2
            status_marker.color.g = 1.0
            status_marker.color.b = 0.3
            status_marker.color.a = 1.0
            status_marker.text = "ALL 50 WAYPOINTS COMPLETED!"
        marker_array.markers.append(status_marker)

        self.marker_pub.publish(marker_array)

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_step_time).nanoseconds * 1e-9
        self.last_step_time = now

        # Fallback dead reckoning kinematics
        if not self.has_odom and dt > 0.0:
            self.pose_x += self.last_cmd.linear.x * math.cos(self.pose_th) * dt
            self.pose_y += self.last_cmd.linear.x * math.sin(self.pose_th) * dt
            self.pose_th += self.last_cmd.angular.z * dt
            self.pose_th = math.atan2(math.sin(self.pose_th), math.cos(self.pose_th))

        # Always broadcast TF and visualization markers
        self._broadcast_tf(now)
        self._publish_markers(now)

        # If all waypoints completed, keep broadcasting TF but hold stop
        if self.idx >= len(self.waypoints):
            if not self.is_completed:
                self.cmd_pub.publish(Twist())
                self.is_completed = True
                self.get_logger().info('All waypoints reached. Holding position in RViz.')
            return

        wx, wy, wtheta = self.waypoints[self.idx]

        # Boundary check
        if not self._inside_boundary(wx, wy, self.boundary):
            self.get_logger().warn(
                'Waypoint %d (%.2f, %.2f) outside boundary – skipping' % (self.idx, wx, wy))
            self.idx += 1
            return

        # Simple smooth P-controller
        dx = wx - self.pose_x
        dy = wy - self.pose_y
        dist = math.hypot(dx, dy)

        if dist > 0.08:
            target_angle = math.atan2(dy, dx)
            angle_err = math.atan2(
                math.sin(target_angle - self.pose_th),
                math.cos(target_angle - self.pose_th)
            )

            k_lin = 0.5
            k_ang = 1.5

            twist = Twist()
            if abs(angle_err) > 0.6:
                # Turn in place if heading error is large
                twist.linear.x = 0.0
                twist.angular.z = max(min(k_ang * angle_err, 1.2), -1.2)
            else:
                twist.linear.x = min(k_lin * dist, 0.40)
                twist.angular.z = max(min(k_ang * angle_err, 1.0), -1.0)

            self.last_cmd = twist
            self.cmd_pub.publish(twist)
        else:
            # Arrived at waypoint
            self.get_logger().info('Reached waypoint %d: (%.2f, %.2f)' % (self.idx, wx, wy))
            self.last_cmd = Twist()
            self.cmd_pub.publish(self.last_cmd)
            self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

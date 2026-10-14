import csv
import math
import os
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy


class WaypointFollower(Node):
    """Simple twist-based follower that respects a box boundary."""

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
        self.get_logger().info('Waypoints loaded: %d' % len(self.waypoints))
        self.get_logger().info('Boundary: %s' % self.boundary)

        # --- Publishers -------------------------------------------------
        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        # --- Timer -------------------------------------------------------
        self.timer = self.create_timer(0.1, self.step)  # 10 Hz

    # -----------------------------------------------------------------
    @staticmethod
    def _read_waypoints(path: str):
        """Read CSV with columns x,y,theta (theta optional). Returns list of (x,y,theta)."""
        rows = []
        if not os.path.isfile(path):
            raise FileNotFoundError('Waypoints file not found: %s' % path)
        with open(path, newline='') as f:
            reader = csv.reader(f)
            for r in reader:
                if not r:
                    continue
                x = float(r[0])
                y = float(r[1])
                theta = float(r[2]) if len(r) > 2 else 0.0
                rows.append((x, y, theta))
        return rows

    @staticmethod
    def _read_boundary(path: str):
        """Read CSV with four numbers: x_min,x_max,y_min,y_max."""
        if not os.path.isfile(path):
            raise FileNotFoundError('Boundary file not found: %s' % path)
        with open(path, newline='') as f:
            reader = csv.reader(f)
            row = next(reader)
        # Expect exactly four values
        vals = [float(v) for v in row]
        if len(vals) != 4:
            raise ValueError('Boundary CSV must have exactly 4 values (x_min,x_max,y_min,y_max)')
        return {
            'x_min': vals[0],
            'x_max': vals[1],
            'y_min': vals[2],
            'y_max': vals[3],
        }

    @staticmethod
    def _inside_boundary(x: float, y: float, b: dict) -> bool:
        return (b['x_min'] <= x <= b['x_max'] and
                b['y_min'] <= y <= b['y_max'])

    def step(self):
        # If all waypoints processed, stop and shutdown timer
        if self.idx >= len(self.waypoints):
            self.cmd_pub.publish(Twist())
            self.timer.cancel()
            self.get_logger().info('All waypoints reached. Stopping.')
            return

        wx, wy, wtheta = self.waypoints[self.idx]

        # Boundary check
        if not self._inside_boundary(wx, wy, self.boundary):
            self.get_logger().warn(
                'Waypoint %d (%.2f, %.2f) outside boundary – skipping' % (self.idx, wx, wy))
            self.idx += 1
            return

        # --- Simple P-controller (position error only) -----------------
        # NOTE: In a real setup you would subscribe to /odom and use real pose.
        # Here we assume the robot is at (0,0) for demo purposes.
        dx = wx - 0.0
        dy = wy - 0.0
        dist = math.hypot(dx, dy)

        k_lin = 0.5
        k_ang = 1.0

        twist = Twist()
        if dist > 0.05:   # threshold to consider arrived
            twist.linear.x = k_lin * dist
            # steer towards waypoint angle (ignoring current orientation)
            twist.angular.z = k_ang * (wtheta - 0.0)
        else:
            # Arrived at waypoint, move to next
            self.get_logger().info('Reached waypoint %d: (%.2f, %.2f)' % (self.idx, wx, wy))
            self.idx += 1
            return

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

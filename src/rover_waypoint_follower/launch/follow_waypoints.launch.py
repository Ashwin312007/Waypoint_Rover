"""Launch file for the rover waypoint follower.

Starts robot_state_publisher (using the four_wheeled_robot_urdf), RViz2 with the
default config, and the waypoint follower node. Waypoint and boundary CSV paths
can be overridden via launch arguments.

Usage
-----
ros2 launch rover_waypoint_follower follow_waypoints.launch.py \
  waypoints_csv:=/home/user/waypoints.csv \
  boundary_csv:=/home/user/boundary.csv
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the URDF of the four-wheeled robot (package must be installed).
    four_wheeled_pkg = get_package_share_directory('four_wheeled_robot_urdf')
    urdf_path = os.path.join(four_wheeled_pkg, 'urdf', 'four_wheeled_robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # ---- Launch arguments -------------------------------------------------
    declare_waypoints_arg = DeclareLaunchArgument(
        'waypoints_csv',
        default_value=os.path.join(
            get_package_share_directory('rover_waypoint_follower'),
            'scripts', 'waypoints.csv'
        ),
        description='Path to waypoints CSV file (format: x,y,theta optional)'
    )
    declare_boundary_arg = DeclareLaunchArgument(
        'boundary_csv',
        default_value=os.path.join(
            get_package_share_directory('rover_waypoint_follower'),
            'scripts', 'boundary.csv'
        ),
        description='Path to boundary CSV file (format: x_min,x_max,y_min,y_max)'
    )

    # ---- Robot state publisher -------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ---- RViz2 ------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',
                   os.path.join(
                       get_package_share_directory('rover_waypoint_follower'),
                       'rviz', 'view_robot.rviz'
                   )]
    )

    # ---- Waypoint follower node -----------------------------------------
    wp_node = Node(
        package='rover_waypoint_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        parameters=[
            {'waypoints_csv': LaunchConfiguration('waypoints_csv')},
            {'boundary_csv' : LaunchConfiguration('boundary_csv')}
        ]
    )

    return LaunchDescription([
        declare_waypoints_arg,
        declare_boundary_arg,
        robot_state_publisher_node,
        rviz_node,
        wp_node,
    ])

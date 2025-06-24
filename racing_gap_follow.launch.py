# racing_gap_follow_launch.py - Launch Gazebo + RViz + Final Adaptive FTG Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # مسیر فایل roboworks.launch.py از ریپوی دانشگاه
    robotverseny_path = os.path.expanduser('~/ros2_ws/src/robotverseny_gazebo24/robotverseny_bringup/launch')
    roboworks_launch = os.path.join(robotverseny_path, 'roboworks.launch.py')

    return LaunchDescription([

        # اجرای محیط شبیه‌سازی roboworks + RViz + پل ارتباطی
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(roboworks_launch),
            launch_arguments={'rviz': LaunchConfiguration('rviz', default='true')}.items()
        ),

        # اجرای نود نهایی ترکیبی Adaptive FTG
        Node(
            package='megoldas_sim24',
            executable='final_cmd_vel_publisher',
            name='final_cmd_vel_publisher',
            output='screen',
            parameters=[{
                'safety_radius': 2.0,
                'max_steering_angle': 0.52,
                'steering_sensitivity': 0.7,
                'min_speed': 0.5,
                'max_speed': 1.0,
                'min_lookahead': 0.8,
                'max_lookahead': 2.0,
                'k_lookahead': 0.05,
                'max_lidar_distance': 5.0
            }]
        )
    ])

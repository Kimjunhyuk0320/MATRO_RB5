#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 공통 설정값
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    lidar_frame = LaunchConfiguration('frame_id', default='laser')

    # RViz 설정파일 경로
    rviz_config_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('frame_id', default_value=lidar_frame),

        # RPLIDAR 노드
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': 115200,
                'frame_id': lidar_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen'
        ),

        # 모터 컨트롤러 노드
        Node(
            package='MATRO_RB5',
            executable='motor_controller',  # 실행할 .py 이름 (확인 필요)
            name='motor_controller',
            output='screen'
        ),

        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])

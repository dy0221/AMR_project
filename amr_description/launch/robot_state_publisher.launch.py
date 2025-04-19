#!/usr/bin/env python3
"""
ros2 launch amr_description robot_state_publisher.launch.py 

ros2 run joint_state_publisher_gui joint_state_publisher_gui 

ros2 run rviz2 rviz2

위에 command 3개로 urdf확인 가능
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 런치파일 실행 시 외부에서 값을 받을 수 있도록 설정
    # 시뮬레이션 시간 사용 여부 (디폴트는 False)
    """
    use_sim_time := false	시스템 시간(진짜 시간)을 사용
    use_sim_time := true	시뮬레이터(Gazebo 등)에서 제공하는 시간 /clock 토픽을 사용
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    # ros2 control 사용 여부 (디폴트는 true)
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # 패키지 경로, xacro
    pkg_path = os.path.join(get_package_share_directory("amr_description"))
    xacro_file = os.path.join(pkg_path, "description", "amr_urdf.xacro")

    # xacro 파일을 처리하여 robot_description을 생성
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # 파라미터 설정
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )
    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'),
            DeclareLaunchArgument(
                'use_ros2_control',
                default_value='true',
                description='Use ros2_control if true'),

            robot_state_publisher,
        ]
    )
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 노드 실행 시 config_file_path 파라미터 전달
    return LaunchDescription([
        Node(
            package='ip_camera_test',                    # 패키지 이름
            executable='ip_webcam_publisher',                    # 실행할 노드 파일
            name='ip_webcam_camera',                          # 노드 이름
            parameters=[{'pkg_src_path': '/home/bjh/ros2_ws/src/ip_camera_test'}],  # 경로를 파라미터로 전달
            output='screen'                          # 출력 설정
        ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dwa_node = Node(
        package='dwa_planner',
        executable='dwa_node',
        name='dwa_planner',
        output='screen',
        parameters=['/home/naitik/ros2_ws/src/dwa_planner/params/params.yaml']  

    return LaunchDescription([dwa_node])

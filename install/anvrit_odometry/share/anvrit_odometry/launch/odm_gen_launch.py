from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='anvrit_odometry',
            executable='odm_gen1',
            output='screen'
        ),
        
        Node(
            package='anvrit_odometry',
            executable='ticks_publisher',
            output='screen'
        ),
        
    ])

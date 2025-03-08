# robot_viz.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    # Directly use the absolute path to the .xacro file
    xacro_file = '/home/lalit/cyb_ws/src/Anvrit/anvrit_description/urdf/anvrit.urdf.xacro'  # Replace with your actual path
    
    # Use xacro to convert the .xacro file to a URDF
    urdf_content = subprocess.check_output(['xacro', xacro_file])
    
    # Convert bytes to string and set the URDF as a parameter
    urdf_str = urdf_content.decode('utf-8')

    # Return launch description
    return LaunchDescription([
        # Start RViz2 with configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # parameters=[{'use_sim_time': True}],  # Assuming you're using simulation time
            # arguments=['-d', '/path/to/your/rviz/config/file.rviz']  # Optional: load a custom RViz config file
        ),
        
        # Start the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_str}]
        ),
        
        # Start the odometry node (if you have one to publish odometry data)
        # Node(
        #     package='your_odometry_package',
        #     executable='your_odometry_node',
        #     name='odometry_node',
        #     output='screen',
        #     remappings=[('/odometry', '/odometry')]  # Make sure this matches your odometry topic
        # ),
    ])

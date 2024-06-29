import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]

    # Construct the ROS 1 launch commandi
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_bringup", "humanoid_robot_intelligence_control_system_bringup.launch"] + args

    # Execute the launch command
    subprocess.call(roslaunch_command)


def ros2_launch_description():
    # Create a list to hold all nodes to be launched
    nodes_to_launch = []
    
    # Add the HUMANOID_ROBOT Manager launch file
    nodes_to_launch.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '$(find humanoid_robot_intelligence_control_system_manager)/launch/',
            'humanoid_robot_intelligence_control_system_manager.launch.py'
        ])
    ))
    
    # Add the UVC camera node
    nodes_to_launch.append(Node(
        package='usb_cam',
        executable='usb_cam_node',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 1280,
            'image_height': 720,
            'framerate': 30,
            'camera_frame_id': 'cam_link',
            'camera_name': 'camera'
        }]
    ))
    
    # Return the LaunchDescription containing all nodes
    return LaunchDescription(nodes_to_launch)


if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)

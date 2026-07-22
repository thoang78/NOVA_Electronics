from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Bridges the Pico's USB serial link into real ROS 2 topics.
        # Requires: sudo apt install ros-<distro>-micro-ros-agent
        # (or build micro-ros-agent from source if not packaged for your distro)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', '/dev/ttyPICO', '-b', '115200'
            ],
            output='screen',
            name='micro_ros_agent'
        ),

        Node(
            package='robot_bringup',
            executable='dynamixel_node',
            name='dynamixel_node',
            output='screen',
        ),

        Node(
            package='robot_bringup',
            executable='wheel_kinematics_node',
            name='wheel_kinematics_node',
            output='screen',
        ),
    ])

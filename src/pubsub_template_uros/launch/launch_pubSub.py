import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # Avvia il micro-ROS agent (UDP6)
    ld.add_action(ExecuteProcess(
        cmd=['bash', '-c', 'source ~/Desktop/agent/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp6 --port 8888'], #change the path of the microRos Agent
        output='screen',
    ))

    # Nodo Talker
    ld.add_action(Node(
        package='pubsub_template_uros',
        executable='talker',
        name='ros2_talker',
        output='screen',
    ))

    # Nodo Listener
    ld.add_action(Node(
        package='pubsub_template_uros',
        executable='listener',
        name='ros2_listener',
        output='screen',
    ))

    return ld

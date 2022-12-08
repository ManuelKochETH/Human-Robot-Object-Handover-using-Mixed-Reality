from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file that launches all necessary ros nodes
    """
    return LaunchDescription([
        Node(
            package='custom_ctrl',
            executable='calibration',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='custom_ctrl',
            executable='hand_tracking_processor',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='custom_ctrl',
            executable='unity_ros_transform',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='custom_ctrl',
            executable='arm_kinematics',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='custom_ctrl',
            executable='safety_check',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='custom_ctrl',
            executable='robot_controller',
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='custom_ctrl',
        #     executable='task_controller',
        #     output='screen',
        #     emulate_tty=True
        # ),

    ])


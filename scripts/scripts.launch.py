from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='raspi',
            executable='move_front_wheels',
            name='front_wheels',
            output='screen'
        ),

        Node(
            package='raspi',
            executable='move_rear_wheels',
            name='rear_wheels',
            output='screen'
        ),

        Node(
            package='raspi',
            executable='inv_kin',
            name='inv_kin',
            output='screen'
        ),
    ])


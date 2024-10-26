import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')


    bento_teleop = Node(
        package='bento_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[ PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'bento_teleop.yaml' ]) ],
        output='screen',
        emulate_tty=True,
    )

    joystick = Node(
        package='joy_linux',
        executable='joy_linux_node',
    )

    image_republish = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        remappings=[
            ('in/compressed', PathJoinSubstitution([ '/', robot_namespace, "cam1/camera_node_1/image_raw/compressed"]) ),
            ('out', "image_repub")
        ],
        arguments=['compressed', 'raw'],
    )


    zbar = Node(
        package='zbar_ros',
        executable='barcode_reader',
        name='barcode',
        parameters=[{'image_transport': 'compressed'}],
	remappings=[('image', 'image_repub')],
    )

    tunnelvision = Node(
        package='TunnelVision',
        executable='process_QR',
        name='qr_antiduplicate',
        output='both',
        emulate_tty=True,
    )

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt',
        arguments=['--perspective-file', PathJoinSubstitution([ '/', 'launch-content', 'rviz.perspective'])],
        output='log',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='bento',
            description='set namespace for robot nodes'
        ),
        GroupAction(
        actions=[
            PushRosNamespace( [LaunchConfiguration("robot_namespace"), '_opr'] ),
            bento_teleop,
            joystick,
            image_republish,
            zbar,
            tunnelvision,
            rqt,
        ])
    ])

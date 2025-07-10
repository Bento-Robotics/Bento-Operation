import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace

from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')


    bento_teleop = Node(
        package='bento_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[ # schaeufele has 2 flipper motorcontrollers, most other robots only have wheels, so 0 as default
                     {'rpm_override_count': PythonExpression(["2 if ('", robot_namespace, "' == 'schaeufele') else 0"  ]) },
                     {PathJoinSubstitution([ '/', 'launch-content', 'parameters', 'bento_teleop.yaml' ])} ],
        output='screen',
        emulate_tty=True,
        namespace=robot_namespace
    )

    joystick = Node(
        package='joy_linux',
        executable='joy_linux_node',
        emulate_tty=True,
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
        emulate_tty=True,
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
        emulate_tty=True,
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
            joystick,
            image_republish,
            zbar,
            tunnelvision,
            rqt,
        ]),
        bento_teleop,
    ])

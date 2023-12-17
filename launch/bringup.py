from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from neato_tag.game import NEATO_TAG


FIRST_SENSOR_PORT = 7777
FIRST_CAMERA_PORT = 8888


def generate_launch_description():
    interfaces_launch_file_dir = os.path.join(get_package_share_directory('neato2_gazebo'), 'launch')

    # Connect to each of the neatos
    # (including this does not bring up the camera connection; connect to neatos separately)
    launch_neatos = [
        GroupAction(actions=[PushRosNamespace(namespace=(robot_name := f'robot{idx}')),
           Node(
                package='neato_node2',
                executable='neato_node',
                name='neato_driver',
                parameters=[{"use_udp": "true"},
                            {"udp_port": str(FIRST_SENSOR_PORT + idx)},
                            {"robot_name": robot_name},
                            {"host": host}],
                output='screen'
           ),
           Node(
                package='fix_scan',
                executable='fix_scan',
                name='fix_scan',
                parameters=[{"robot_name": robot_name}]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([interfaces_launch_file_dir, '/robot_state_publisher.py']),
                launch_arguments={'tf_prefix': robot_name}.items()
            ),
            Node(
                package='neato_node2',
                executable='setup_udp_stream',
                name='udp_stream_setup',
                parameters=[{"receive_port": (udp_video_port := str(FIRST_CAMERA_PORT + idx))},
                            {"width": 1024},
                            {"height": 768},
                            {"fps": 30},
                            {"host": host}],
                output='screen'
            ),
            Node(
                package='gscam',
                executable='gscam_node',
                parameters=[
                    {'preroll': True},
                    {'camera_name': 'camera'},
                    {'use_gst_timestamps': False},
                    {'frame_id': 'camera'},
                    {'gscam_config': ['udpsrc port=',
                                      udp_video_port,
                                      ' ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert']}
                ]
            )
        ])
    for idx, host in enumerate(NEATO_TAG.hosts)]

    # Run the camera detector nodes
    camera_detector_nodes = [
        Node(
            package='neato_tag',
            executable='camera_detector',
            name=f'camera_detector_{idx}',
            remappings=[
                ('camera/image_raw', f'robot{idx}/camera/image_raw'),
                ('neatos_in_camera', f'robot{idx}/neatos_in_camera')
            ]
        )
    for idx in range(NEATO_TAG.num_players)]

    # Run the tagging nodes
    tagging_nodes = [
        Node(
            package='neato_tag',
            executable='tagging',
            name=f'tagging_{idx}',
            parameters=[{
                'neato_id': idx,
                'start_as_it': idx == 0
            }],
            remappings=[
                ('camera/image_raw', f'robot{idx}/camera/image_raw'),
                ('bump', f'robot{idx}/bump')
            ]
        )
    for idx in range(NEATO_TAG.num_players)]

    # Run the navigator nodes
    navigator_nodes = [
        Node(
            package='neato_tag',
            executable='navigator',
            name=f'navigator_{idx}',
            parameters=[
                {'neato_id': idx},
                {'start_as_it': idx == 0}
            ],
            remappings=[
                ('scan', f'robot{idx}/scan'),
                ('neatos_in_camera', f'robot{idx}/neatos_in_camera'),
                ('cmd_vel', f'robot{idx}/cmd_vel')
            ]
        )
    for idx in range(NEATO_TAG.num_players)]

    return LaunchDescription(tagging_nodes + camera_detector_nodes + navigator_nodes)

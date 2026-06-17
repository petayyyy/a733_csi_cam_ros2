from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('a733_csi_cam_ros2')

    return LaunchDescription([
        DeclareLaunchArgument('device',           default_value='/dev/video8'),
        DeclareLaunchArgument('topic',            default_value='/camera/image_raw'),
        DeclareLaunchArgument('fps',              default_value='30'),
        DeclareLaunchArgument('in_size',          default_value='1280x960'),
        DeclareLaunchArgument('resize_w',         default_value='320'),
        DeclareLaunchArgument('resize_h',         default_value='240'),
        DeclareLaunchArgument('qos_reliable',     default_value='true'),
        DeclareLaunchArgument('enable_isp',       default_value='true'),
        DeclareLaunchArgument('frame_id',         default_value='camera_optical'),
        DeclareLaunchArgument('calibration_file', default_value=''),

        Node(
            package='a733_csi_cam_ros2',
            executable='imx219_camera_node',
            name='imx219_camera_node',
            output='screen',
            parameters=[{
                'device':           LaunchConfiguration('device'),
                'topic':            LaunchConfiguration('topic'),
                'fps':              LaunchConfiguration('fps'),
                'in_size':          LaunchConfiguration('in_size'),
                'resize_w':         LaunchConfiguration('resize_w'),
                'resize_h':         LaunchConfiguration('resize_h'),
                'qos_reliable':     LaunchConfiguration('qos_reliable'),
                'enable_isp':       LaunchConfiguration('enable_isp'),
                'frame_id':         LaunchConfiguration('frame_id'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
    ])

import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('a733_csi_cam_ros2')
    config_dir = os.path.join(pkg_share, 'config')

    camera_tfs = {}
    params_path = os.path.join(config_dir, 'params.yaml')
    print(f'[a733_csi_cam_ros2] Loading camera TF from: {params_path}', file=sys.stderr)
    if os.path.exists(params_path):
        try:
            with open(params_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            for i in range(3):
                key = f'camera_tf_{i}'
                src = cfg.get(key, {})
                src = src.get('ros__parameters', src)
                camera_tfs[str(i)] = {
                    'parent_frame': str(src.get('parent_frame', 'base_link')),
                    'xyz': [str(x) for x in src.get('xyz', [0.0, 0.0, 0.0])],
                    'rpy': [str(r) for r in src.get('rpy', [0.0, 0.0, 0.0])],
                }
            print(f'[a733_csi_cam_ros2] Loaded camera_tfs: {camera_tfs}', file=sys.stderr)
        except Exception as e:
            print(f'[a733_csi_cam_ros2] Error loading params.yaml: {e}', file=sys.stderr)

    camera_id = LaunchConfiguration('camera_id')

    ld_items = [
        DeclareLaunchArgument('device',           default_value='/dev/video8'),
        DeclareLaunchArgument('topic',            default_value='/camera/image_raw'),
        DeclareLaunchArgument('fps',              default_value='30'),
        DeclareLaunchArgument('in_size',          default_value='1280x960'),
        DeclareLaunchArgument('resize_w',         default_value='320'),
        DeclareLaunchArgument('resize_h',         default_value='240'),
        DeclareLaunchArgument('qos_reliable',     default_value='true'),
        DeclareLaunchArgument('enable_isp',       default_value='true'),
        DeclareLaunchArgument('camera',           default_value='1',
                              description='Alias for camera_id'),
        DeclareLaunchArgument('camera_id',        default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument(
            'frame_id',
            default_value=PythonExpression(["'camera_optical_' + str('", camera_id, "')"]),
        ),
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
    ]

    for cam_id, tf_cfg in camera_tfs.items():
        ld_items.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=[
                    TextSubstitution(text='camera_tf_'),
                    TextSubstitution(text=cam_id),
                ],
                condition=IfCondition(EqualsSubstitution(camera_id, cam_id)),
                arguments=[
                    tf_cfg['xyz'][0], tf_cfg['xyz'][1], tf_cfg['xyz'][2],
                    tf_cfg['rpy'][0], tf_cfg['rpy'][1], tf_cfg['rpy'][2],
                    tf_cfg['parent_frame'],
                    LaunchConfiguration('frame_id'),
                ],
                output='screen',
            )
        )

    return LaunchDescription(ld_items)

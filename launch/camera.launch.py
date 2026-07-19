import os
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
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

    source_type = LaunchConfiguration('source_type')
    sensor = LaunchConfiguration('sensor')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    camera_id = LaunchConfiguration('camera_id')
    resize_w = LaunchConfiguration('resize_w')
    resize_h = LaunchConfiguration('resize_h')
    video_device = LaunchConfiguration('device')
    pixel_format = LaunchConfiguration('format')
    io_mode = LaunchConfiguration('io_mode')
    qos_reliable = LaunchConfiguration('qos_reliable')

    topic = PythonExpression(["'/camera_' + str('", camera_id, "') + '/image_raw'"])
    frame_id = PythonExpression(["'camera_optical_' + str('", camera_id, "')"])
    in_size = PythonExpression(["str('", width, "') + 'x' + str('", height, "')"])
    calibration_file = PythonExpression([
        "'", config_dir, "/' + str('", sensor,
        "') + '_' + str('", width,
        "') + 'x' + str('", height, "') + '.yaml'",
    ])

    ld_items = [
        DeclareLaunchArgument('source_type',      default_value='v4l2',
                              description='Capture backend. A733 supports v4l2.'),
        DeclareLaunchArgument('sensor',           default_value='imx219',
                              description='Sensor name, used for calibration file naming.'),
        DeclareLaunchArgument('width',            default_value='1280'),
        DeclareLaunchArgument('height',           default_value='960'),
        DeclareLaunchArgument('fps',              default_value='30'),
        DeclareLaunchArgument('camera',           default_value='1',
                              description='Alias for camera_id'),
        DeclareLaunchArgument('camera_id',        default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument('resize_w',         default_value='0',
                              description='Output width (0 = same as capture)'),
        DeclareLaunchArgument('resize_h',         default_value='0',
                              description='Output height (0 = same as capture)'),
        DeclareLaunchArgument('video_device',     default_value='/dev/video8',
                              description='V4L2 capture device'),
        DeclareLaunchArgument('device',           default_value=LaunchConfiguration('video_device'),
                              description='Legacy alias for video_device'),
        DeclareLaunchArgument('format',           default_value='BGR24',
                              description='V4L2 pixel format. A733 node currently captures BGR24.'),
        DeclareLaunchArgument('io_mode',          default_value='mmap',
                              description='V4L2 io-mode. A733 node currently uses mmap buffers.'),
        DeclareLaunchArgument('qos_reliable',     default_value='true'),
        DeclareLaunchArgument('enable_isp',       default_value='true'),
        DeclareLaunchArgument('flip_180',         default_value='true',
                              description='Rotate image 180 deg (camera mounted upside-down)'),
        DeclareLaunchArgument('topic',            default_value=topic),
        DeclareLaunchArgument('frame_id',         default_value=frame_id),
        DeclareLaunchArgument('in_size',          default_value=in_size,
                              description='Legacy WIDTHxHEIGHT alias for width/height'),
        DeclareLaunchArgument('calibration_file', default_value=calibration_file),

        # Camera static TF from launch_system (config/frames/<frame>.yaml,
        # full_system_real passes tf_x..tf_yaw). If tf_x is empty, fall back
        # to camera_tf_<id> from params.yaml (legacy behavior).
        DeclareLaunchArgument('tf_parent_frame',  default_value='base_link'),
        DeclareLaunchArgument('tf_x',             default_value=''),
        DeclareLaunchArgument('tf_y',             default_value='0.0'),
        DeclareLaunchArgument('tf_z',             default_value='0.0'),
        DeclareLaunchArgument('tf_roll',          default_value='0.0'),
        DeclareLaunchArgument('tf_pitch',         default_value='0.0'),
        DeclareLaunchArgument('tf_yaw',           default_value='0.0'),

        Node(
            package='a733_csi_cam_ros2',
            executable='imx219_camera_node',
            name=PythonExpression(["'camera_' + str('", camera_id, "')"]),
            output='screen',
            parameters=[{
                'source_type':      source_type,
                'sensor':           sensor,
                'topic':            LaunchConfiguration('topic'),
                'fps':              fps,
                'width':            width,
                'height':           height,
                'in_size':          LaunchConfiguration('in_size'),
                'video_device':     video_device,
                'device':           video_device,
                'format':           pixel_format,
                'io_mode':          io_mode,
                'resize_w':         resize_w,
                'resize_h':         resize_h,
                'qos_reliable':     qos_reliable,
                'enable_isp':       LaunchConfiguration('enable_isp'),
                'frame_id':         LaunchConfiguration('frame_id'),
                'flip_180':         LaunchConfiguration('flip_180'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
    ]

    def make_static_tf(context):
        frame_id_val = LaunchConfiguration('frame_id').perform(context)
        cam_id_val = LaunchConfiguration('camera_id').perform(context)
        tf_x = LaunchConfiguration('tf_x').perform(context).strip()

        if tf_x:
            # Explicit tf_* from launch_system (frames/<frame>.yaml) is the
            # source of truth. Use the new CLI syntax (--roll/--pitch/--yaw)
            # to avoid the ordering trap of the legacy positional arguments
            # (which take yaw pitch roll).
            return [Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'camera_tf_{cam_id_val}',
                arguments=[
                    '--x', tf_x,
                    '--y', LaunchConfiguration('tf_y').perform(context),
                    '--z', LaunchConfiguration('tf_z').perform(context),
                    '--roll', LaunchConfiguration('tf_roll').perform(context),
                    '--pitch', LaunchConfiguration('tf_pitch').perform(context),
                    '--yaw', LaunchConfiguration('tf_yaw').perform(context),
                    '--frame-id', LaunchConfiguration('tf_parent_frame').perform(context),
                    '--child-frame-id', frame_id_val,
                ],
                output='screen',
            )]

        # Fallback: camera_tf_<id> from params.yaml (positional argument order
        # kept as before: x y z, then the rpy values into yaw pitch roll slots).
        tf_cfg = camera_tfs.get(cam_id_val)
        if not tf_cfg:
            return []
        return [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'camera_tf_{cam_id_val}',
            arguments=[
                tf_cfg['xyz'][0], tf_cfg['xyz'][1], tf_cfg['xyz'][2],
                tf_cfg['rpy'][0], tf_cfg['rpy'][1], tf_cfg['rpy'][2],
                tf_cfg['parent_frame'],
                frame_id_val,
            ],
            output='screen',
        )]

    ld_items.append(OpaqueFunction(function=make_static_tf))

    return LaunchDescription(ld_items)

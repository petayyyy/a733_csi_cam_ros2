# Contributing

Thanks for improving `a733_csi_cam_ros2`.

## Development Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/petayyyy/a733_csi_cam_ros2.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select a733_csi_cam_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Before Opening a Pull Request

- Keep the package focused on Orange Pi Zero 3W / Allwinner A733 CSI capture.
- Test on real hardware when changing V4L2 or ISP behavior.
- Keep `README.md` and `CHANGELOG.md` in sync with user-visible changes.
- Prefer small, reviewable commits.

## Useful Checks

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select a733_csi_cam_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

On hardware:

```bash
sudo modprobe vin_v4l2
ros2 launch a733_csi_cam_ros2 camera.launch.py device:=/dev/video8 topic:=/camera_1/image_raw
ros2 topic hz /camera_1/image_raw
```

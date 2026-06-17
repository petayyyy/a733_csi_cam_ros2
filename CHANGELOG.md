# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses semantic versioning where practical.

## [1.0.0] - 2026-06-18

### Added

- Initial standalone ROS 2 Humble package for IMX219 on Orange Pi Zero 3W / Allwinner A733.
- Direct V4L2 `VIDEO_CAPTURE_MPLANE` capture path with `mmap` buffers.
- `sensor_msgs/Image` publication in `bgr8`.
- `sensor_msgs/CameraInfo` publication through `camera_info_manager`.
- Optional output resize through OpenCV.
- Optional Allwinner ISP 3A support through `AWIspApi`.
- Launch file and placeholder IMX219 calibration YAML.
- Repository README, MIT license, Git ignore rules and GitHub Actions CI.

# pypylon_image_stream

`pypylon_image_stream` is a ROS 2 package for streaming images from Basler cameras using the [pypylon](https://github.com/basler/pypylon) library.

## Features

- Connects to Basler cameras via pypylon
- Publishes camera images to ROS 2 topics
- Configurable image parameters

## Requirements

- ROS 2 (Humble or newer)
- Python 3
- [pypylon](https://github.com/basler/pypylon) (`pip install pypylon`)
- Udev rule:
  - `SUBSYSTEM=="usb", ATTR{idVendor}=="2676", MODE="0666", GROUP="plugdev"`

## Installation

```bash
pip install pypylon
cd ~/ros2_ws/src
git clone https://github.com/ArendJan/pypylon_image_stream.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pypylon_image_stream pypylon_image_stream_node
```
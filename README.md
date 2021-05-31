# Reachy 2021 focus

ROS2 package handling autofocus of Reachy's cameras.

**ROS2 version: Foxy**

Dependencies: [reachy_msgs](https://github.com/pollen-robotics/reachy_msgs), [opencv](https://opencv.org/)

How to install:

```bash
cd ~/reachy_ws/src
git clone https://github.com/pollen-robotics/reachy_focus.git
cd ~/reachy_ws/
colcon build --packages-select reachy_focus
```

## Subscribed topics
* **/left_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))[[camera_focus](https://github.com/pollen-robotics/reachy_focus/blob/master/reachy_focus/camera_focus.py)] - Compressed image from the left camera.<br> Default size: 640x480, default fps: 30.

* **/right_image** ([sensor_msgs/msg/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html))[[camera_focus](https://github.com/pollen-robotics/reachy_focus/blob/master/reachy_focus/camera_focus.py)] - Compressed image from the right camera.<br> Default size: 640x480, default fps: 30.

## Services

* **/set_focus_state** ([reachy_msgs/srv/SetFocusState.srv](https://github.com/marjoriePaillet/reachy_msgs/blob/integration/srv/SetFocusState.srv))[[camera_focus](https://github.com/pollen-robotics/reachy_focus/blob/master/reachy_focus/camera_focus.py)] - Start or stop the autofocus for a given camera.

## Launch file
* **camera_focus.launch.py** - Launch camera focus node. 
---
This package is part of the ROS2-based software release of the version 2021 of Reachy.

Visit [pollen-robotics.com](https://pollen-robotics.com) to learn more or visit [our forum](https://forum.pollen-robotics.com) if you have any questions.

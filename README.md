Teleoperation System (Meta Quest 3 → Kinova Gen3)
Overview

This project implements a full teleoperation pipeline using a Meta Quest 3 (TeleVuer) VR interface to control a Kinova Gen3 robot arm in real time.
Communication between VR and robot is achieved via shared memory, and robot-side control uses the Kortex API.

The system includes:

VR → Shared Memory Producer (test2.py)
Kinova Teleoperation Controller (twist2.py) 
ROS2 Image Viewer for Kinova Vision(ros2 launch kinova_vision kinova_vision.launch)

1. Environment Setup

Required:

Ubuntu 22.04
ROS 2 Humble
Kinova Gen3 + Kortex API Python examples installed
(twist2.py expects the path: /home/icatheon/Kinova-kortex2_Gen3_G3L/api_python/examples)

Python dependencies:
numpy
scipy
opencv-python
pyrealsense2 (if using Realsense)
televuer
Built-in: multiprocessing, csv, time

Shared Memory Layout
cmd (float64 × 12)
Index	Meaning
0–2	VR linear velocity estimate (vx, vy, vz)
3–5	VR angular velocity (unused in final version)
6	right trigger value (0.0–1.0) – used for gripper control
7	right A button (0 or 1)
8	right B button (0 or 1)
9	right thumbstick x
10	right thumbstick y
11	left A button (reserved, e.g., for camera switching)
televuer_img

Stereo buffer: (480, 1280, 3) RGB, uint8
Used for VR video display (optional).

3.  Kinova Vision Camera via ROS2
Start Kinova Vision:
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch kinova_vision kinova_vision.launch.py



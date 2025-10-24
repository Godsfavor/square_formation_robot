# Square Formation Behavior

A ROS 2 package for coordinating 4 TurtleBot3 robots in a square formation that rotate positions synchronously when receiving left/right signals.

## Overview

This project implements a behavior where 4 robots arranged in a square formation rotate to their neighboring positions when triggered by signals on `/left` or `/right` topics.

```
Initial Square:          After /right signal:     After /left signal:
R1 ---- R2              R4 ---- R1               R2 ---- R3
|        |              |        |               |        |
R4 ---- R3              R3 ---- R2               R1 ---- R4
(Clockwise rotation)                             (Counter-clockwise)
```

## Prerequisites

- **ROS 2 Humble** (or compatible version)
- **Gazebo** (Gazebo Classic 11)
- **Python 3.8+**
- **TurtleBot3 packages** (provided via multi_turtlebot_sim)

# Square Formation Robot – Multi-TurtleBot3 ROS2 Package

![TurtleBot3 Square Formation](https://raw.githubusercontent.com/Godsfavor/square_formation_robot/main/docs/square_formation_demo.gif)

> **Make 4 TurtleBot3 robots move in a perfect square and rotate on command!**  
> Real-robot tested | ROS2 Humble | Simple to use

---

## What This Does

This package lets **4 TurtleBot3 robots**:
- Form a **1x1 meter square** (adjustable size)
- **Rotate clockwise or counter-clockwise** when you send a signal
- Move smoothly using odometry feedback
- Work together using **static TF transforms** so all robots share a common reference frame

Perfect for demos, education, or swarm robotics experiments!

---

## Requirements

| Item | Requirement |
|------|-------------|
| **ROS2** | Humble (Ubuntu 22.04) |
| **Robots** | 4 x TurtleBot3 Burger (LDS-01) |
| **Network** | All on same WiFi (e.g., `PIROBOTNET`) |
| **PC** | Ubuntu 22.04 with ROS2 installed |

> Warning: All devices **must use the same `ROS_DOMAIN_ID`**

---

## Folder Structure

square_formation_robot/
├── launch/
│   └── real_robots_demo.launch.py     ← Launch file for real robots
├── square_formation_behavior/
│   ├── square_formation_controller.py ← Main controller
│   └── send_signal.py                 ← Send /left or /right commands
├── rviz/square_formation.rviz         ← RViz config
└── README.md



---

## Setup (Step-by-Step)

### 1. **Connect to Robot WiFi**
```bash
# On your PC
Connect to: PIROBOTNET or PIROBOTNET6_5G

SSH into Each Robot

ssh ubuntu@192.168.0.XXX  # aire1
ssh ubuntu@192.168.0.XXX  # agua2
ssh ubuntu@192.168.0.XXX  # fuego4
ssh ubuntu@192.168.0.XXX  # tierra3 

Launch Each Robot (One per SSH terminal)

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01

# Replace namespace with robot name
ros2 launch turtlebot3_bringup robot.launch.py namespace:=aire1


Repeat for:

namespace:=agua2
namespace:=fuego4
namespace:=tierra3

## Launch the Formation Controller

On your main PC:

# Clone & build
cd ~/ros2_ws/src
git clone https://github.com/Godsfavor/square_formation_robot.git
cd ~/ros2_ws
colcon build --packages-select square_formation_robot
source install/setup.bash

# Set domain ID
export ROS_DOMAIN_ID=1

# Launch!
ros2 launch square_formation_robot real_robots_demo.launch.py

You should see:

=== Square Formation Controller (Real Robots) ===
Robots: ['tierra3', 'fuego4', 'aire1', 'agua2']
Waiting for /left or /right signals...

Send Rotation Commands
In a new terminal:

# Rotate counter-clockwise
ros2 topic pub /left example_interfaces/Empty "{}" --once

# Rotate clockwise
ros2 topic pub /right example_interfaces/Empty "{}" --once

Or run the helper script:

ros2 run square_formation_behavior send_signal.py left
ros2 run square_formation_behavior send_signal.py right

## Visualize in RViz

rviz2 -d $(ros2 pkg prefix square_formation_robot)/share/square_formation_robot/rviz/square_formation.rviz

You’ll see:

4 robot models
Odometry paths
Laser scans
TF tree with tierra3/odom as root


# Check TF Tree (Important!)

ros2 run tf2_tools view_frames
evince frames.pdf

You should see:

tierra3/odom (root)
├── fuego4/odom
├── aire1/odom
└── agua2/odom

## Parameters (Edit in Launch File)

Parameter,Default,Description
robot_namespaces,"['tierra3', 'fuego4', 'aire1', 'agua2']",Robot names
square_size,1.0,Side length in meters
position_tolerance,0.10,Stop when within 10 cm
angle_tolerance,0.20,~11.5° alignment
min_robots,2,Allow testing with fewer


Troubleshooting

Problem,Fix
Robots not moving,Check tierra3 is launched and ROS_DOMAIN_ID=X everywhere
Only 1 TF link in view_frames,Run export ROS_DOMAIN_ID=1 before view_frames
cmd_vel not received,Check topic: ros2 topic echo /aire1/cmd_vel
Laser frame not namespaced,Source fixed driver: source ~/ros2_ws/install/setup.bash

Fixed LDS-01 Laser Bug
Some robots publish /scan with frame base_scan (no namespace).
Fix: Use patched driver from RobInLabUJI

# On each robot
cd ~/ros2_ws/src
git clone https://github.com/RobInLabUJI/hls_lfcd_lds_driver.git
cd ~/ros2_ws
colcon build --packages-select hls_lfcd_lds_driver
source install/setup.bash


Now /aire1/scan has frame aire1/base_scan


Credits & References

TurtleBot3 ROS2 Bringup
Multi-Robot Namespace Guide
RobInLabUJI LDS Fix


## Star This Repo If You Like It!
Made with love for robotics education
Godsfavor – 2025



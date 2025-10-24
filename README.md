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

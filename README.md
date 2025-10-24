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

### Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-nav-msgs
```

## Installation

### 1. Create Workspace

```bash
mkdir -p ~/mrs3_ws/src
cd ~/mrs3_ws/src
```

### 2. Clone Required Repositories

```bash
# Clone the multi-robot simulator
git clone https://github.com/RobInLabUJI/multi_turtlebot_sim.git

# Clone this repository
git clone <YOUR_GITHUB_REPO_URL> square_formation_behavior
```

### 3. Build the Workspace

```bash
cd ~/mrs3_ws
colcon build --symlink-install
source install/setup.bash
```

## Package Structure

```
square_formation_behavior/
├── launch/
│   └── square_formation_demo.launch.py    # Main launch file
├── rviz/
│   └── square_formation_simple.rviz       # RViz configuration
├── square_formation_behavior/
│   ├── __init__.py
│   ├── square_formation_controller.py     # Main controller node
│   └── send_signal.py                     # Signal publisher utility
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Usage

### Launch the Complete System

This will start Gazebo with 4 robots in a square formation and the controller node:

```bash
cd ~/mrs3_ws
source install/setup.bash
ros2 launch square_formation_behavior square_formation_demo.launch.py
```

### Send Rotation Signals

Open a new terminal and send rotation commands:

```bash
cd ~/mrs3_ws
source install/setup.bash

# Rotate clockwise
ros2 run square_formation_behavior send_signal right

# Rotate counter-clockwise
ros2 run square_formation_behavior send_signal left
```

**Alternative method** - publish directly to topics:

```bash
# Clockwise rotation
ros2 topic pub --once /right example_interfaces/msg/Empty

# Counter-clockwise rotation
ros2 topic pub --once /left example_interfaces/msg/Empty
```

### Visualize in RViz

```bash
cd ~/mrs3_ws
source install/setup.bash
rviz2 -d src/square_formation_behavior/rviz/square_formation_simple.rviz
```

**RViz Configuration:**
- **Fixed Frame:** `robot1/odom`
- **Displays:** TF, LaserScans (all 4 robots)
- The robots will appear as colored laser scans with TF frames showing their positions

## Recording Bag Files

### Record Simulation Data

```bash
cd ~/mrs3_ws
source install/setup.bash

# Record required topics
ros2 bag record -o square_formation_gazebo \
  /robot1/odom /robot2/odom /robot3/odom /robot4/odom \
  /tf /tf_static \
  /left /right
```

### Playback Recorded Data

```bash
# Play back the bag file
ros2 bag play square_formation_gazebo

# In another terminal, launch RViz to visualize
rviz2 -d src/square_formation_behavior/rviz/square_formation_simple.rviz
```

## How It Works

### Controller Node (`square_formation_controller.py`)

The main controller node:
1. **Subscribes to:**
   - `/left` and `/right` topics (Empty messages)
   - `/robot1/odom`, `/robot2/odom`, `/robot3/odom`, `/robot4/odom` (odometry)

2. **Publishes to:**
   - `/robot1/cmd_vel`, `/robot2/cmd_vel`, `/robot3/cmd_vel`, `/robot4/cmd_vel` (velocity commands)

3. **Behavior:**
   - Tracks current positions of all 4 robots
   - When signal received, calculates target positions for rotation
   - Moves all robots synchronously to their new positions
   - Uses proportional control for smooth movement

### Movement Algorithm

```python
# For clockwise (/right):
robot1 → robot2's position
robot2 → robot3's position
robot3 → robot4's position
robot4 → robot1's position

# For counter-clockwise (/left):
robot1 → robot4's position
robot4 → robot3's position
robot3 → robot2's position
robot2 → robot1's position
```

### Square Formation Positions

Default 2m × 2m square:
- **Robot1:** (1.0, 1.0) - Top Right
- **Robot2:** (1.0, -1.0) - Bottom Right
- **Robot3:** (-1.0, -1.0) - Bottom Left
- **Robot4:** (-1.0, 1.0) - Top Left

## Configuration

### Modify Square Size

Edit `square_formation_behavior/square_formation_controller.py`:

```python
self.square_positions = {
    'robot1': {'x': 2.0, 'y': 2.0},   # Change coordinates
    'robot2': {'x': 2.0, 'y': -2.0},
    'robot3': {'x': -2.0, 'y': -2.0},
    'robot4': {'x': -2.0, 'y': 2.0}
}
```

### Modify Movement Speed

In the same file, adjust control parameters:

```python
# Linear velocity (line ~167)
cmd.linear.x = min(0.2, distance * 0.5)  # Change 0.2 for max speed

# Angular velocity (line ~165)
cmd.angular.z = 1.0 * angle_diff  # Change 1.0 for rotation speed
```

## Troubleshooting

### Yellow Background in RViz

**Cause:** Transform issues between robot frames  
**Solution:** Make sure Fixed Frame is set to `robot1/odom` in RViz Global Options

### Robots Not Moving

**Check if controller is running:**
```bash
ros2 node list | grep formation_controller
```

**Check if signals are received:**
```bash
ros2 topic echo /left
# or
ros2 topic echo /right
```

### Robots Spawn at Same Location

**Cause:** Gazebo initialization timing  
**Solution:** Wait a few seconds after launch before sending signals

### Build Errors

```bash
# Clean and rebuild
cd ~/mrs3_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/left` | `example_interfaces/msg/Empty` | Trigger counter-clockwise rotation |
| `/right` | `example_interfaces/msg/Empty` | Trigger clockwise rotation |
| `/robot[1-4]/odom` | `nav_msgs/msg/Odometry` | Robot odometry data |
| `/robot[1-4]/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/robot[1-4]/scan` | `sensor_msgs/msg/LaserScan` | Robot laser scan data |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms |

## Nodes

| Node | Executable | Description |
|------|-----------|-------------|
| `formation_controller` | `square_formation_controller` | Main controller for square formation |
| `signal_publisher` | `send_signal` | Utility to send rotation signals |

## Parameters

The controller node accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim_time` | bool | `true` | Use simulation time from Gazebo |

## Dependencies

- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types
- `example_interfaces` - Example message types
- `tf2_ros` - Transform library

## Future Improvements

- [ ] Add obstacle avoidance during rotation
- [ ] Implement formation shape changes (circle, line, etc.)
- [ ] Add velocity synchronization for smoother movement
- [ ] Support for more than 4 robots
- [ ] Add ROS 2 actions for asynchronous control
- [ ] Implement collision detection

## License

[Specify your license here, e.g., Apache 2.0, MIT, etc.]

## Contributors

[Add your name and contributors]

## Acknowledgments

- [RobInLabUJI](https://github.com/RobInLabUJI) for the multi_turtlebot_sim package
- TurtleBot3 simulation packages

## Contact

[Your contact information or course information]

---

**Project Assignment:** Multi-Robot Systems Course  
17th October 2024

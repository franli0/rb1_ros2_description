# RB1 ROS2 Description Package

This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base with ROS2 Control integration for both differential drive and elevator control.

## Overview

The package provides:
- Complete URDF/XACRO robot description for the RB1 robot
- ROS2 Control integration with Gazebo simulation
- Differential drive controller for robot mobility
- Position controller for elevator platform
- Automatic controller spawning and configuration

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url>
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Usage

### Starting the Simulation

Launch the complete simulation with automatic controller spawning:

```bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
```

This will:
1. Start Gazebo with an empty world
2. Spawn the RB1 robot
3. Load the gazebo_ros2_control plugin
4. Automatically spawn and activate all controllers:
   - `joint_state_broadcaster` (at 130 seconds)
   - `rb1_base_controller` (at 130 seconds)  
   - `elevator_controller` (at 130 seconds)

### Verifying the Setup

Check that all controllers are active:

```bash
# List all active controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Verify topics are available
ros2 topic list | grep -E "(cmd_vel|elevator)"
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
rb1_base_controller[diff_drive_controller/DiffDriveController] active
elevator_controller[position_controllers/JointGroupPositionController] active
```

### Robot Movement Commands

#### Differential Drive Control

Move the robot using velocity commands:

```bash
# Move forward
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Combined movement (forward + turn)
ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Stop the robot
ros2 topic pub --once /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Keyboard Teleoperation

Control the robot with keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb1_base_controller/cmd_vel_unstamped
```

### Elevator Control Commands

Control the elevator platform height:

```bash
# Check if elevator controller is active
ros2 control list_controllers | grep elevator

# Raise elevator to maximum height (30cm)
ros2 topic pub --once /elevator_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3]}"

# Lower elevator to minimum height (ground level)
ros2 topic pub --once /elevator_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Set elevator to middle position (15cm)
ros2 topic pub --once /elevator_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.15]}"

# Set elevator to 1/4 height (7.5cm)
ros2 topic pub --once /elevator_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.075]}"

# Set elevator to 3/4 height (22.5cm)
ros2 topic pub --once /elevator_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.225]}"
```

#### Monitor Elevator Position

Check current elevator position:

```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor elevator controller output
ros2 topic echo /elevator_controller/controller_state
```

## Troubleshooting

### Controllers Not Loading

If controllers fail to spawn automatically:

```bash
# Manually spawn controllers
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager
ros2 run controller_manager spawner rb1_base_controller --controller-manager /controller_manager  
ros2 run controller_manager spawner elevator_controller --controller-manager /controller_manager
```

### Robot Not Moving

1. Verify controllers are active:
   ```bash
   ros2 control list_controllers
   ```

2. Check if hardware interfaces are claimed:
   ```bash
   ros2 control list_hardware_interfaces
   ```

### Elevator Not Responding

1. Check if elevator controller is loaded:
   ```bash
   ros2 control list_controllers | grep elevator
   ```

2. Verify the elevator joint exists:
   ```bash
   ros2 topic echo /joint_states | grep elevator
   ```

## Configuration Files

- **Launch file**: `launch/rb1_ros2_xacro.launch.py`
- **Robot description**: `xacro/rb1_ros2_base.urdf.xacro`
- **Controller config**: `config/rb1_controller.yaml`

## ROS2 Control Architecture

The robot uses ROS2 Control with the following components:

- **Hardware Interface**: `gazebo_ros2_control/GazeboSystem`
- **Controllers**:
  - `diff_drive_controller/DiffDriveController`
  - `position_controllers/JointGroupPositionController`
  - `joint_state_broadcaster/JointStateBroadcaster`
# Palm Harvester — ROS 2 Mobile Manipulator

A tracked mobile manipulator designed for autonomous palm fruit harvesting. Skid-steer base with a 5-DOF arm, sickle blade end-effector, and a full sensor suite (LiDAR, depth camera, IMU, GPS). Simulated in Gazebo Harmonic with gz_ros2_control; SLAM via slam_toolbox.

## Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF/xacro model, RViz display launch |
| `robot_simulation` | Gazebo launch files, world SDFs, controller config, scan_relay node |
| `robot_slam` | slam_toolbox async mapping/localization |
| `robot_navigation` | Nav2 (planned) |
| `robot_interfaces` | Custom messages/services (planned) |
| `robot_services` | High-level services (planned) |
| `robot_tasks` | Task sequencing (planned) |
| `robot_vision` | Perception pipeline (planned) |

## Robot Specs

**Base**
- Chassis: 2.0 m × 1.0 m × 0.4 m, 180 kg
- Tracked differential drive, 1.22 m track separation, 0.25 m track radius
- Max speed: 1.5 m/s forward, 1.0 rad/s yaw

**Arm (5-DOF)**
| Joint | Type | Range | Effort |
|---|---|---|---|
| `arm_pan_joint` | revolute (Z) | ±160° | 120 N·m |
| `boom_pitch_joint` | revolute (Y) | 0° – 75° | 800 N·m |
| `boom_extend_joint` | prismatic (X) | 0 – 2.0 m | 400 N |
| `wrist_pitch_joint` | revolute (Y) | ±90° | 40 N·m |
| `wrist_roll_joint` | revolute (X) | ±90° | 20 N·m |

**End-effector:** sickle blade (`blade_spin_joint`, continuous, up to ~314 rad/s)

**Sensors**
- Ouster LiDAR — 12 m range, mounted on sensor mast
- RealSense D455 — depth camera + point cloud
- RGB camera — mast top
- IMU
- GPS

## Prerequisites

ROS 2 Jazzy + Gazebo Harmonic. Install binary dependencies:

```bash
sudo apt install \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-velocity-controllers \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro
```

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

**URDF viewer** (no Gazebo):
```bash
ros2 launch robot_description display.launch.py
```

**Gazebo simulation — empty world:**
```bash
ros2 launch robot_simulation gazebo.launch.py
```

**Gazebo simulation — farm world:**
```bash
ros2 launch robot_simulation farm_sim.launch.py
```

Launch arguments (both sim launches):

| Argument | Default | Description |
|---|---|---|
| `world` | package default | Path to SDF world file |
| `x` / `y` / `z` | 0 / 0 / 0.15 | Spawn position (m) |
| `yaw` | 0.0 | Spawn heading (rad) |
| `rviz` | true | Open RViz alongside Gazebo |

**SLAM (run alongside a sim launch):**
```bash
ros2 launch robot_slam slam.launch.py
```

## Key Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe |
| `/odom` | `nav_msgs/Odometry` | publish |
| `/joint_states` | `sensor_msgs/JointState` | publish |
| `/scan` | `sensor_msgs/LaserScan` | publish |
| `/imu/data` | `sensor_msgs/Imu` | publish |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | publish |
| `/camera/image_raw` | `sensor_msgs/Image` | publish |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | publish |
| `/clock` | `rosgraph_msgs/Clock` | publish (sim only) |

**Arm control** — send `FollowJointTrajectory` goals to:
```
/arm_trajectory_controller/follow_joint_trajectory
```

**Blade speed** — publish `std_msgs/Float64MultiArray` to:
```
/blade_velocity_controller/commands
```

## Controllers

Managed by `gz_ros2_control` via `robot_simulation/config/ros2_control.yaml`:

| Controller | Type |
|---|---|
| `joint_state_broadcaster` | Publishes all joint states |
| `diff_drive_controller` | `/cmd_vel` → tracks, publishes `/odom` + TF |
| `arm_trajectory_controller` | 5-DOF position trajectory control |
| `blade_velocity_controller` | Blade spin velocity |

## Worlds

| File | Description |
|---|---|
| `empty.sdf` | Flat plane, no obstacles |
| `farm_world.sdf` | Palm plantation rows |
| `obstacles.sdf` | Generic obstacle course |

## License

Apache-2.0

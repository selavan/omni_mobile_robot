# omni_mobile_robot
## A* and RRT* Global Path Tracking for SLAM-enabled Mecanum Wheel Robot

This repository contains the ROS 2 implementation for my thesis project. It focuses on autonomous navigation, SLAM, and path tracking (A*/RRT*) for an omnidirectional mecanum wheel robot.

## 🛠️ Prerequisites

* **OS:** Ubuntu 20.04 (Focal Fossa)
* **ROS Version:** ROS 2 Foxy Fitzroy
* **Hardware:**
    * Mecanum Wheel Mobile Robot chassis
    * LiDAR (LDS / RTF)
    * IMU (HFI-A9)
    * PS4 Controller (DualShock 4)

## 📥 Installation

1. **Create a workspace:**
    ```bash
    mkdir -p ~/mobile_robot_ws/src
    cd ~/mobile_robot_ws/src
    ```

2. **Clone the repository:**
    ```bash
    git clone https://github.com/selavan/omni_mobile_robot.git
    ```

3. **Install dependencies:**
    ```bash
    cd ~/mobile_robot_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. **Build the project:**
    ```bash
    colcon build
    source install/setup.bash
    ```

---

## 🚀 Usage

### 1. Launch Entire System (Recommended)
To start all drivers, sensors, and the control system simultaneously:
```bash
ros2 launch omni_robot omni_robot_launch.py
```

### 2. Launch Individual Components

#### 2.1. Interface with motor by "can"
- pub(from encoder): /motor_feedback
- sub(motor command): /publish_motor
```bash
ros2 run can_motor can_motor_node
```

#### 2.2. Body velocity
- pub: /publish_motor
- sub: /cmd_vel
```bash
ros2 run omni_robot body_velocity_node 
```

#### 2.3. PS4 Controller
- pub: /raw_report
```bash
ros2 run ds4_driver ds4_driver_node.py
```

#### 2.4. Teleoperation 
- pub: /cmd_vel
- sub: /raw_report
```bash
ros2 run ds4_teleop controller_node
```

#### 2.5. LiDAR 
- pub:/scan
```bash
ros2 launch rtf_lds_driver hlds_laser.launch.py
```

#### 2.6. IMU
- pub: imu/data_raw
- pub: imu/mag
```bash
$ ros2 run hfi_a9 hfi_a9_node
```

#### 2.7. Odom
- pub: /odom
- sub: /motor_feedback, /yaw_oem
```bash
ros2 run omni_robot odom_node
```
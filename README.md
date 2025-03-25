
# 🐆 Jaguar Odom Package

This repository contains a ROS 2 package for mobile robot odometry fusion using micro-ROS on an ESP32 (or compatible MCU) and a Python-based odometry node. It integrates encoder feedback, IMU data, LiDAR, and prepares for future visual odometry with an Astra Pro Plus camera. The system is designed for accurate real-time odometry and mapping using `slam_toolbox`.

---

## 🧩 Features

### Micro-ROS Arduino Node
- Publishes encoder-based velocity and RPMs with direction info.
- Publishes full 9-DOF IMU data using the Adafruit BNO055.
- Runs as a FreeRTOS task and publishes via Wi-Fi using UDP.

### ROS 2 Python Node (`encoder_odom_node`)
- Subscribes to IMU and encoder messages.
- Applies complementary filtering for angular velocity.
- Uses exponential moving average (EMA) filters.
- Computes and publishes odometry in `nav_msgs/Odometry`.
- Broadcasts TF transform: `odom → base_footprint`.

### Launch File
- Starts SLAM with `slam_toolbox`.
- Launches the LiDAR node and static transform publisher.
- Launches odometry node.

---

## 🛠 Hardware Setup

| Component          | Description                        |
|--------------------|------------------------------------|
| MCU                | ESP32 with micro-ROS support       |
| IMU                | Adafruit BNO055                    |
| Encoders           | 722 PPR with quadrature encoding   |
| Motors             | Gear ratio 34:20                   |
| Wheels             | Diameter: 18 cm                    |
| LiDAR              | SLLidar A2M12                      |
| Visual Odometry    | Astra Pro Plus (future integration)|

---

## 📦 Software Dependencies

- ROS 2 (Humble or newer recommended)
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)
- `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `tf_transformations`
- `slam_toolbox`
- `sllidar_ros2` package

---

## 🔧 Installation

1. **Clone this repository into your ROS 2 workspace:**

    ```bash
    cd ~/ros2_ws/src
    git clone <repo_url>
    ```

2. **Install dependencies:**

    ```bash
    rosdep install --from-paths . --ignore-src -r -y
    ```

3. **Build the workspace:**

    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

---

## 🚀 Upload Firmware (ESP32)

1. **Connect to Wi-Fi:**

    In your Arduino sketch (already configured in the firmware):

    ```cpp
    set_microros_wifi_transports("robot_1", "robot123", "192.168.0.129", 8888);
    ```

2. **Upload code using Arduino IDE:**

    - Make sure the `micro_ros_arduino` library is installed.
    - Flash the complete firmware script provided in this package.

---

## 🧠 ROS 2 Node Execution

Once everything is compiled and firmware uploaded, run:

```bash
ros2 launch jaguar_odom jaguar_combined.launch.py
```

This will:

- Launch the SLLidar node.
- Start the `encoder_odom_node` for odometry estimation.
- Start SLAM with `slam_toolbox`.
- Publish a static transform from `base_footprint` to `laser`.

---

## 📈 Topics Overview

| Topic           | Type                          | Description                             |
|------------------|-------------------------------|-----------------------------------------|
| `/imu`           | `sensor_msgs/msg/Imu`         | Raw IMU data from BNO055                |
| `/motors_info`   | `std_msgs/msg/String`         | Parsed string with velocities and RPMs  |
| `/odom`          | `nav_msgs/msg/Odometry`       | Computed robot odometry                 |
| `/tf`            | `tf2_msgs/TFMessage`          | Transform: `odom → base_footprint`      |
| `/scan`          | `sensor_msgs/msg/LaserScan`   | LiDAR data from SLLidar                 |

---

## 📐 Coordinate Frames

```
map
  ↓ (via SLAM Toolbox)
odom
  ↓ (from odometry)
base_footprint
  ↓ (static transform to LiDAR)
laser
```

---

## 📅 Future Work

**Current Stage:**

- Adding visual odometry using Astra Pro Plus camera and integrating it into an Extended Kalman Filter (EKF) for sensor fusion.
- Planning to create a 3D map using RTAB-Map with LiDAR + camera fusion.

---

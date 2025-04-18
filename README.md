
# Ginkgo USB-CAN Robot Arm Control

This repository contains the necessary setup to simulate and control a robotic arm using ROS 2 and a Ginkgo USB-CAN interface. The architecture is based on a MoveIt simulation (from a SolidWorks model) and real-time hardware execution via CAN.

---

## 🔧 Project Structure

```
├── dicerox_arm_confv2/         # URDF package generated from SolidWorks
│   ├── urdf/                   # Contains .urdf files and meshes
│   ├── config/                 # MoveIt configuration files
│   ├── launch/                 # MoveIt launch files
│   ├── CMakeLists.txt          # MODIFIED for ROS 2 compatibility
│   └── package.xml             # MODIFIED for ROS 2 compatibility
│
├── can_teleop_odrive/          # ROS 2 node to convert JointStates to CAN commands
│   ├── ControlCAN.py           # ctypes wrapper for Ginkgo driver (Linux only)
│   └── joint_to_can.py         # Main node sending data to ODrive and servos
│
└── arduino_servo_driver/       # Microcontroller code (Arduino/ESP32)
    └── servo_driver.ino        # Reads CAN commands and drives servos via PCA9685
```

---

## 🦾 MoveIt Simulation

1. The robotic arm was modeled in SolidWorks and exported to URDF.
2. **MoveIt Setup Assistant** was used to generate the simulation and planning files.
3. **Note:** The generator produces ROS 1-style `CMakeLists.txt` and `package.xml`, which were manually adapted to ROS 2 (`ament_cmake` and dependencies updated).
4. The simulation publishes joint states to the `/joint_states` topic.

---

## 🛠️ CAN Communication

- A **Ginkgo USB-CAN** interface is used.
- Communication is set at **500 kbps**.
- The `ControlCAN.py` file defines the ctypes interface to interact with the hardware using Linux 64-bit libraries.

---

## 🔄 ROS 2 Node – `joint_to_can`

This node:
- Subscribes to the `/joint_states` topic.
- Translates joint values to:
  - `CMD_SET_INPUT_POS` for an **ODrive** motor (position in revolutions).
  - Simple commands for servo channels (angle in degrees).
- Sends CAN frames with a short delay (`0.05s`) to ensure message delivery.

---

## ⚙️ Microcontroller – Servo Driver

A microcontroller (Arduino or ESP32) with an **MCP2515** CAN module:
- Listens for CAN messages with ID `0x200`.
- Interprets the first byte as the servo channel and the second byte as the angle (0–180°).
- Uses a **PCA9685** module (I²C) to control multiple servos simultaneously.

### Frame example:
```
ID: 0x200
Data[0]: Servo channel (0–15)
Data[1]: Angle in degrees (0–180)
```

---

## 🧪 System Verification

1. Launch the MoveIt simulation.
2. Run the `joint_to_can` node.
3. Observe the real motors and servos moving in sync with the simulation.

---

## 📌 To-do

- [ ] Remove unused references to Windows and Mac libraries in `ControlCAN.py`.
- [ ] Add support for multiple ODrive motors (parameterized).
- [ ] Extend servo control to additional channels and joints.
- [ ] Document the installation of dependencies like `Adafruit_PWMServoDriver`, `mcp2515`, and `libginkgo`.

---

## 📚 Resources

- [Ginkgo USB-CAN API Library Manual](http://www.viewtool.com/demo/Ginkgo/Documents/Ginkgo_USB-CAN_Interface_API_Library_Instruction_v1.2.zip)
- [Ginkgo Baudrate Calculator](http://www.viewtool.com/demo/download/Ginkgo2/Ginkgo_USB-CAN_Baudrate_Windows.rar)



# PPM → Arm + Gripper Control

This project allows you to control a robotic arm with three joints (base, shoulder, elbow) and a gripper using PPM signals from an RC receiver (like the FS-iA6B). The system reads PPM from an Arduino and sends real-time commands through a Python GUI to a microcontroller like an ESP32 on the robot.

---

## 🧠 How It Works

### PPM Signal

PPM (Pulse Position Modulation) is a **digital signal** that packs all RC transmitter channels into **a single wire**.

- Each channel is represented by a pulse length (typically 1000–2000 µs).
- A long pulse (> 2100 µs) indicates the start of a new frame.
- Arduino measures each pulse and reconstructs channel values sequentially.

You only need **one input pin** to read **all channels**, no need for separate pins per channel.

### Arduino

- Uses interrupt on pin 2 to read the PPM signal.
- Interprets channels 2 (Z), 3 (Y), 4 (X) and 6 (gripper) as controls.
- Applies deadzones to filter noise and uses incremental step logic for smooth control.
- Maps each channel to internal variables `b`, `s`, `e` (in radians), and `t` (for the gripper).
- Sends a CSV message every 50ms in the format:

```csv
b_val,s_val,e_val,t_val
```

### Python Interface

- Reads the CSV serial data and sends a **JSON command** directly to the robot:

```json
{
  "T": 102,
  "base": 0.0,
  "shoulder": 0.0,
  "elbow": 1.57,
  "hand": 3.14,
  "spd": 0,
  "acc": 10
}
```

- JSON is sent directly to the ESP32 without conversions.
- GUI allows toggling motion, setting speed and acceleration, and visualizing live joint values.

---

## ⚙️ Requirements

### Hardware

- RC Transmitter + Receiver 
- Arduino signal decoding
- ESP32 or compatible microcontroller on the robot
- 3 motors + gripper

### Software

- Python 3.x
- `customtkinter`
- `pyserial`

Install dependencies:

```bash
pip install customtkinter pyserial
```

---

## 📡 Enable PPM Mode on FlySky FS-i6

1. Hold **"OK"** to enter the hidden menu.
2. Go to **System Setup → RX Setup → Output mode**.
3. Select **"Output"** and switch to **PPM**.
4. Save and exit.

✅ You only need **one signal wire** from the PPM output of the receiver to Arduino pin D2.

---

## 🧪 Troubleshooting

- Make sure the switch (channel 5) is **down** (> 1200) to enable the arm.
- Confirm PPM mode is enabled on your transmitter.
- Use Serial Monitor to verify the Arduino is outputting values.
- Check COM ports in the Python GUI.

---

import os
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import can_teleop_odrive.ControlCAN as can
from ctypes import byref, c_uint

# ─── Constants ──────────────────────────────────────────────────────────────
TIMING = {
    1000: dict(SJW=1, BS1=2, BS2=1, BRP=9),
    800 : dict(SJW=1, BS1=3, BS2=1, BRP=12),
    500 : dict(SJW=1, BS1=4, BS2=1, BRP=12),
    250 : dict(SJW=1, BS1=6, BS2=1, BRP=18),
    125 : dict(SJW=1, BS1=6, BS2=1, BRP=36),
    100 : dict(SJW=1, BS1=7, BS2=2, BRP=60),
}

CMD_SET_INPUT_POS  = 0x00C
COBID_SERVO        = 0x200

# wait time after each CAN frame
CAN_SEND_DELAY = 0.05  # seconds

def make_cobid(node, cmd):
    return (node << 5) | (cmd & 0x1F)

def float_to_bytes_le(f):
    """Pack Python float into 4‑byte little‑endian IEEE‑754"""
    import struct
    return struct.pack('<f', f)

# ─── Node Definition ────────────────────────────────────────────────────────
class JointToCan(Node):
    def __init__(self):
        super().__init__('joint_to_can')

        # ROS params
        self.declare_parameter('can_ch',    0)
        self.declare_parameter('kbps',      500)
        self.declare_parameter('odrv_node', 2)
        self.declare_parameter('servo0_ch', 0)
        self.declare_parameter('servo2_ch', 2)

        can_ch         = self.get_parameter('can_ch').value
        kbps           = self.get_parameter('kbps').value
        self.odrv_node = self.get_parameter('odrv_node').value
        self.servo0_ch = self.get_parameter('servo0_ch').value
        self.servo2_ch = self.get_parameter('servo2_ch').value

        # 1) Open and init CAN
        DEV, IDX = can.VCI_USBCAN2, 0
        ret = can.VCI_OpenDevice(DEV, IDX, 0)
        if ret not in (0,1):
            self.get_logger().error("VCI_OpenDevice failed")
            raise SystemExit

        if kbps not in TIMING:
            self.get_logger().error(f"kbps must be one of: {tuple(TIMING)}")
            raise SystemExit

        t = TIMING[kbps]
        cfg = can.VCI_INIT_CONFIG_EX()
        cfg.CAN_Mode, cfg.CAN_ABOM, cfg.CAN_NART, cfg.CAN_RFLM, cfg.CAN_TXFP, cfg.CAN_RELAY = 0,0,0,0,1,0
        cfg.CAN_SJW,   cfg.CAN_BS1, cfg.CAN_BS2, cfg.CAN_BRP = t["SJW"], t["BS1"], t["BS2"], t["BRP"]

        if can.VCI_InitCANEx(DEV, IDX, can_ch, byref(cfg)) != 1:
            self.get_logger().error("VCI_InitCANEx failed")
            raise SystemExit

        if can.VCI_StartCAN(DEV, IDX, can_ch) != 1:
            self.get_logger().error("VCI_StartCAN failed")
            raise SystemExit

        self.can_dev = (DEV, IDX, can_ch)
        self.get_logger().info(f"CAN up on ch{can_ch} @ {kbps}kbps")

        # Subscribe to joint_states
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)

    def send_odrive(self, turns: float):
        DEV, IDX, ch = self.can_dev
        cmd_id = make_cobid(self.odrv_node, CMD_SET_INPUT_POS)
        payload = float_to_bytes_le(turns) + b'\x00'*4

        frame = can.VCI_CAN_OBJ()
        frame.ID         = c_uint(cmd_id)
        frame.SendType   = 0
        frame.RemoteFlag = 0
        frame.ExternFlag = 0
        frame.DataLen    = 8
        for i,b in enumerate(payload):
            frame.Data[i] = b

        if can.VCI_Transmit(DEV, IDX, ch, byref(frame), 1) != 1:
            self.get_logger().warn("Failed to send ODrive frame")
        time.sleep(CAN_SEND_DELAY)

    def send_servo(self, channel: int, angle: int):
        DEV, IDX, ch = self.can_dev

        frame = can.VCI_CAN_OBJ()
        frame.ID         = c_uint(COBID_SERVO)
        frame.SendType   = 0
        frame.RemoteFlag = 0
        frame.ExternFlag = 0
        frame.DataLen    = 8
        frame.Data[0]    = channel & 0xFF
        frame.Data[1]    = angle   & 0xFF
        for i in range(2,8):
            frame.Data[i] = 0

        if can.VCI_Transmit(DEV, IDX, ch, byref(frame), 1) != 1:
            self.get_logger().warn(f"Failed to send servo CH{channel}")
        time.sleep(CAN_SEND_DELAY)

    def cb_joint(self, msg: JointState):
        # Log incoming joint positions (first six only)
        self.get_logger().info(f"← joint_states: {msg.position[:6]}")

        # Need at least 4 joint values
        if len(msg.position) < 4:
            self.get_logger().warn("Not enough joints, skipping")
            return

        # joint 2 → index 1 → ODrive turns (in revolutions)
        turns = (msg.position[1] / (2 * math.pi))*10 #10/transmission
        self.get_logger().info(f"→ ODrive turns (node {self.odrv_node}): {turns:.3f}")
        self.send_odrive(turns)

        # joint 3 → index 2 → servo CH0
        deg3 = int(math.degrees(msg.position[4]))
        self.get_logger().info(f"→ Servo CH{self.servo0_ch}: {deg3}°")
        deg3 = max(0, min(180, deg3))
        self.send_servo(self.servo0_ch, deg3)

        # joint 4 → index 3 → servo CH2
        deg4 = int(math.degrees(msg.position[5]))
        self.get_logger().info(f"→ Servo CH{self.servo2_ch}: {deg4}°")
        deg4 = max(0, min(180, deg4))
        self.send_servo(self.servo2_ch, deg4)

    def destroy_node(self):
        DEV, IDX, _ = self.can_dev
        can.VCI_CloseDevice(DEV, IDX)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JointToCan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

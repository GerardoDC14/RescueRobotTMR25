import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from joint_state_subscriber.roboclaw_3_bak import Roboclaw
from joint_state_subscriber.serial_commands import Cmd

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        # ——— ROS subscription ———
        self.joint1 = 'Joint3'   # → Motor 1
        self.joint2 = 'Joint2'   # → Motor 2
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        # ——— Common parameters ———
        PORT        = '/dev/ttyACM0'
        BAUD        = 115200
        ADDR        = 0x80
        PPR         = 500
        QUAD        = 4
        GEAR_RATIO  = 128.0

        # counts per full joint revolution at the output shaft:
        self.counts_per_joint_rev = PPR * QUAD * GEAR_RATIO

        # motion profile
        self.accel  = 5_000
        self.speed  = 20_000
        self.decel  = 5_000
        self.buffer = 1

        # individual max angles
        self.max1 = 195.0  # Joint3 → Motor 1
        self.max2 = 240.0  # Joint2 → Motor 2

        # ——— Open & configure Roboclaw ———
        self.rc = Roboclaw(PORT, BAUD)
        if not self.rc.Open():
            self.get_logger().error(f"Failed to open {PORT}@{BAUD}")
            rclpy.shutdown()
            return

        # enable quadrature on both channels
        self.rc.SetM1EncoderMode(ADDR, QUAD)
        self.rc.SetM2EncoderMode(ADDR, QUAD)
        time.sleep(0.05)

        self.get_logger().info("RoboClaw ready – listening for joint_states…")

    def listener_callback(self, msg: JointState):
        # process Joint3 → Motor 1
        if self.joint1 in msg.name:
            idx = msg.name.index(self.joint1)
            self._move_motor(
                msg.position[idx],
                self.max1,
                motor=1
            )

        # process Joint2 → Motor 2
        if self.joint2 in msg.name:
            idx = msg.name.index(self.joint2)
            self._move_motor(
                msg.position[idx],
                self.max2,
                motor=2,
                invert=True
            )

    def _move_motor(self, rad, max_deg, motor, invert=False):
        """Convert rad→deg→counts, clamp, optionally invert, then send."""
        deg = abs(math.degrees(rad))
        if deg > max_deg:
            self.get_logger().warn(f"Requested {deg:.1f}° > {max_deg}°, clamping")
            deg = max_deg

        counts = int((deg / 360.0) * self.counts_per_joint_rev)
        if invert:
            counts = -counts

        self.get_logger().info(
            f"→ Moving Joint{motor} to {deg:.1f}° → {counts} counts"
        )

        cmd = (self.rc.SpeedAccelDeccelPositionM1 if motor == 1
               else self.rc.SpeedAccelDeccelPositionM2)

        ok = cmd(
            0x80,
            self.accel,
            self.speed,
            self.decel,
            counts,
            self.buffer
        )
        if not ok:
            self.get_logger().error(f"Failed to send pos cmd to M{motor}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
cmd_vel_bridge.py — differential drive bridge for 2x Dynamixel servos
S1 = left wheel, S2 = right wheel
Nav2 /cmd_vel → /servo1/command + /servo2/command

SIGN CONVENTION (must match odom_node.py):
  LEFT_SIGN  = +1.0  → positive RPM = left wheel forward
  RIGHT_SIGN = -1.0  → positive RPM COMMAND becomes negative at the servo
                        because the right wheel is physically mirrored.
                        The servo will report present_velocity < 0 when going forward.

If the robot moves BACKWARD when you press forward:
  → Swap: LEFT_SIGN = -1.0 and RIGHT_SIGN = +1.0
  → Then also update LEFT_SIGN/RIGHT_SIGN in odom_node.py to match.

If the robot spins instead of going straight:
  → Swap LEFT_SERVO_ID and RIGHT_SERVO_ID (also in odom_node.py).
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_interfaces.msg import DynamixelCommand
from std_srvs.srv import SetBool

# ── Robot geometry ─────────────────────────────────────────────────────────
WHEEL_RADIUS   = 0.033     # metres  — must match odom_node.py
WHEEL_BASE     = 0.230     # metres  — must match odom_node.py

# ── Servo IDs ──────────────────────────────────────────────────────────────
LEFT_SERVO_ID  = 1
RIGHT_SERVO_ID = 2

# ── Motor direction signs ──────────────────────────────────────────────────
# LEFT_SIGN=+1, RIGHT_SIGN=-1 because the right wheel is mounted mirrored.
# These MUST match odom_node.py (left_sign / right_sign parameters).
LEFT_SIGN      = +1.0
RIGHT_SIGN     = -1.0

# Motor trim: if robot drifts LEFT when going forward → increase LEFT_TRIM above 1.0
#             if robot drifts RIGHT when going forward → increase RIGHT_TRIM above 1.0
# Start at 1.0 and adjust in steps of 0.02 until robot goes straight
LEFT_TRIM      = 1.00
RIGHT_TRIM     = 1.00

MAX_RPM        = 250.0
# VEL_SCALE compensates if the servo achieves less RPM than commanded.
# Set to 1.0 first and tune up if the robot is too slow.
VEL_SCALE      = 1.0
TIMEOUT        = 5.0       # seconds without cmd_vel before stopping


def mps_to_rpm(mps: float) -> float:
    return (mps / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0 * VEL_SCALE


def _make_cmd(servo_id: int, rpm: float) -> DynamixelCommand:
    cmd = DynamixelCommand()
    cmd.servo_id             = servo_id
    cmd.operating_mode       = -1          # keep current mode
    cmd.torque_enable        = -1          # keep current torque state
    cmd.goal_velocity        = float(rpm)
    cmd.goal_position        = float('nan')
    cmd.goal_current         = float('nan')
    cmd.goal_pwm             = float('nan')
    cmd.profile_velocity     = float('nan')
    cmd.profile_acceleration = float('nan')
    return cmd


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Publishers
        self.pub_left  = self.create_publisher(
            DynamixelCommand, f'/servo{LEFT_SERVO_ID}/command',  10)
        self.pub_right = self.create_publisher(
            DynamixelCommand, f'/servo{RIGHT_SERVO_ID}/command', 10)

        # Torque service clients
        self.torque_left  = self.create_client(
            SetBool, f'/servo{LEFT_SERVO_ID}/set_torque')
        self.torque_right = self.create_client(
            SetBool, f'/servo{RIGHT_SERVO_ID}/set_torque')

        # Subscriptions — Nav2 sends to /cmd_vel, teleop to /cmd_vel_teleop
        self.create_subscription(Twist, '/cmd_vel',        self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.cmd_vel_cb, 10)

        self.last_cmd_time  = self.get_clock().now()
        self.initialized    = False
        self.motors_running = False

        self.create_timer(1.0, self.init_motors)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            'cmd_vel_bridge ready (dual motor mode: S1=left S2=right)')

    # ── Initialise both servos in VELOCITY mode ───────────────────────────
    def init_motors(self):
        if self.initialized:
            return
        if not self.torque_left.wait_for_service(timeout_sec=0.1):
            return
        if not self.torque_right.wait_for_service(timeout_sec=0.1):
            return

        self.get_logger().info('Setting VELOCITY mode on S1 + S2 ...')

        for servo_id, pub in [(LEFT_SERVO_ID,  self.pub_left),
                               (RIGHT_SERVO_ID, self.pub_right)]:
            cmd = DynamixelCommand()
            cmd.servo_id             = servo_id
            cmd.operating_mode       = 1          # VELOCITY mode
            cmd.torque_enable        = -1
            cmd.goal_velocity        = 0.0
            cmd.goal_position        = float('nan')
            cmd.goal_current         = float('nan')
            cmd.goal_pwm             = float('nan')
            cmd.profile_velocity     = float('nan')
            cmd.profile_acceleration = float('nan')
            pub.publish(cmd)

        import time; time.sleep(0.3)

        for client in (self.torque_left, self.torque_right):
            req = SetBool.Request()
            req.data = True
            client.call_async(req)

        self.initialized = True
        self.get_logger().info('Both motors ready!')

    # ── Convert Twist → wheel RPMs ────────────────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        if not self.initialized:
            return

        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # Standard differential drive kinematics
        v_left  = v - w * WHEEL_BASE / 2.0
        v_right = v + w * WHEEL_BASE / 2.0

        rpm_left  = max(-MAX_RPM, min(MAX_RPM,
                        mps_to_rpm(v_left)  * LEFT_SIGN  * LEFT_TRIM))
        rpm_right = max(-MAX_RPM, min(MAX_RPM,
                        mps_to_rpm(v_right) * RIGHT_SIGN * RIGHT_TRIM))

        # Deadband: servo needs at least ~40 RPM to overcome static friction
        MIN_MOVE_RPM = 30.0
        if 0.0 < abs(rpm_left)  < MIN_MOVE_RPM:
            rpm_left  = math.copysign(MIN_MOVE_RPM, rpm_left)
        if 0.0 < abs(rpm_right) < MIN_MOVE_RPM:
            rpm_right = math.copysign(MIN_MOVE_RPM, rpm_right)

        self.get_logger().debug(
            f'cmd_vel v={v:.3f} w={w:.3f} → '
            f'L={rpm_left:.1f} R={rpm_right:.1f} rpm')

        # Re-enable torque if motors were stopped by watchdog
        if not self.motors_running and (rpm_left != 0.0 or rpm_right != 0.0):
            for client in (self.torque_left, self.torque_right):
                req = SetBool.Request()
                req.data = True
                client.call_async(req)
            self.motors_running = True

        self.pub_left.publish(_make_cmd(LEFT_SERVO_ID,   rpm_left))
        self.pub_right.publish(_make_cmd(RIGHT_SERVO_ID, rpm_right))

    # ── Safety watchdog ───────────────────────────────────────────────────
    def watchdog(self):
        if not self.initialized:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > TIMEOUT and self.motors_running:
            self.get_logger().info('watchdog: no cmd_vel — stopping motors')
            self.pub_left.publish(_make_cmd(LEFT_SERVO_ID,   0.0))
            self.pub_right.publish(_make_cmd(RIGHT_SERVO_ID, 0.0))
            self.motors_running = False


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_left.publish(_make_cmd(LEFT_SERVO_ID,   0.0))
            node.pub_right.publish(_make_cmd(RIGHT_SERVO_ID, 0.0))
            for client in (node.torque_left, node.torque_right):
                req = SetBool.Request()
                req.data = False
                client.call_async(req)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

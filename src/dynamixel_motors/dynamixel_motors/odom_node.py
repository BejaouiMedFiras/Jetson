#!/usr/bin/env python3
"""
odom_node.py — FIXED v2
========================
Key fix: dynamixel_node publishes on /servo1/state + /servo2/state
         (NOT /dynamixel_state). Subscribe to the correct topics.
Robot dims: 60cm × 30cm, ~4kg, wheel_r=0.033, wheelbase=0.160
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from dynamixel_interfaces.msg import DynamixelState, DynamixelCommand

# ── Robot parameters ──────────────────────────────────────────────────
WHEEL_RADIUS_M  = 0.033
WHEEL_BASE_M    = 0.230
ODOM_HZ         = 20.0
DXL_VEL_TO_RPM  = 0.229   # 1 DXL unit = 0.229 RPM

LEFT_MOTOR_ID   = 1
RIGHT_MOTOR_ID  = 2

# Signs: left=+1 (forward=positive), right=-1 (mirrored mounting)
LEFT_SIGN       = +1.0
RIGHT_SIGN      = -1.0

ZERO_TICKS_THRESHOLD = 6   # 2s at 20Hz before switching to CMD fallback

_BIG = 1e6
POSE_COV  = [0.002,0,0,0,0,0, 0,0.002,0,0,0,0, 0,0,_BIG,0,0,0,
             0,0,0,_BIG,0,0,  0,0,0,0,_BIG,0,  0,0,0,0,0,0.05]
TWIST_COV = [0.001,0,0,0,0,0, 0,_BIG,0,0,0,0,  0,0,_BIG,0,0,0,
             0,0,0,_BIG,0,0,  0,0,0,0,_BIG,0,  0,0,0,0,0,0.05]


def rpm_to_mps(rpm):
    return rpm * 2.0 * math.pi * WHEEL_RADIUS_M / 60.0

def yaw_to_quat(yaw):
    q = Quaternion()
    q.x = q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_pub')

        self.declare_parameter('wheel_radius',   WHEEL_RADIUS_M)
        self.declare_parameter('wheel_base',     WHEEL_BASE_M)
        self.declare_parameter('left_motor_id',  LEFT_MOTOR_ID)
        self.declare_parameter('right_motor_id', RIGHT_MOTOR_ID)
        self.declare_parameter('left_sign',      LEFT_SIGN)
        self.declare_parameter('right_sign',     RIGHT_SIGN)

        self.wheel_r  = self.get_parameter('wheel_radius').value
        self.wheel_b  = self.get_parameter('wheel_base').value
        self.left_id  = self.get_parameter('left_motor_id').value
        self.right_id = self.get_parameter('right_motor_id').value
        self.l_sign   = self.get_parameter('left_sign').value
        self.r_sign   = self.get_parameter('right_sign').value

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Publishers ────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry,   '/odom',         10)
        self.js_pub   = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_br    = TransformBroadcaster(self)

        # ── PRIMARY: subscribe to per-servo state topics ───────────
        # dynamixel_node publishes /servo1/state and /servo2/state
        self.create_subscription(
            DynamixelState, f'/servo{self.left_id}/state',
            lambda m: self._state_cb(m, 'left'), be_qos)
        self.create_subscription(
            DynamixelState, f'/servo{self.right_id}/state',
            lambda m: self._state_cb(m, 'right'), be_qos)

        # ── FALLBACK: commanded RPM from bridge ───────────────────
        self.create_subscription(
            DynamixelCommand, f'/servo{self.left_id}/command',
            lambda m: self._cmd_cb(m, 'left'), 10)
        self.create_subscription(
            DynamixelCommand, f'/servo{self.right_id}/command',
            lambda m: self._cmd_cb(m, 'right'), 10)

        # ── State ─────────────────────────────────────────────────
        self.left_vel   = 0.0   # present_velocity (DXL units or rad/s)
        self.right_vel  = 0.0
        self.left_cmd   = 0.0   # goal_velocity RPM
        self.right_cmd  = 0.0

        self.x = self.y = self.yaw = 0.0
        self.left_pos  = 0.0
        self.right_pos = 0.0
        self.last_t    = self.get_clock().now()

        self._zero_ticks   = 0
        self._using_cmd    = False
        self._warn_printed = False
        self._log_once     = False
        self._boot_skip    = 60  # ignorer 60 ticks (3s à 20Hz) au démarrage

        self.create_timer(1.0 / ODOM_HZ, self._tick)

        self.get_logger().info(
            f'odom_node v2 | r={self.wheel_r} b={self.wheel_b} '
            f'| L=/servo{self.left_id}/state R=/servo{self.right_id}/state '
            f'| L_sign={self.l_sign:+} R_sign={self.r_sign:+}')

    def _state_cb(self, msg: DynamixelState, side: str):
        """Receive present_velocity from /servo{id}/state"""
        if not self._log_once:
            self.get_logger().info(
                f'[odom] First state msg side={side} '
                f'servo_id={msg.servo_id} '
                f'present_velocity={msg.present_velocity:.2f} '
                f'operating_mode={msg.operating_mode}')
            self._log_once = True
        if side == 'left':
            self.left_vel  = float(msg.present_velocity)
        else:
            self.right_vel = float(msg.present_velocity)

    def _cmd_cb(self, msg: DynamixelCommand, side: str):
        """Fallback: store commanded RPM"""
        if not hasattr(msg, 'goal_velocity'):
            return
        rpm = float(msg.goal_velocity)
        if math.isnan(rpm):
            return
        if side == 'left':
            self.left_cmd  = rpm
        else:
            self.right_cmd = rpm

    def _tick(self):
        now = self.get_clock().now()
        dt  = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:
            return
        # Ignorer les ticks de démarrage (moteurs encore en coast)
        if self._boot_skip > 0:
            self._boot_skip -= 1
            self.left_vel = 0.0
            self.right_vel = 0.0
            return

        # ── Decide velocity source ────────────────────────────────
        cmd_nonzero = abs(self.left_cmd)  > 1.0 or abs(self.right_cmd) > 1.0
        fb_nonzero  = abs(self.left_vel)  > 0.01 or abs(self.right_vel) > 0.01

        if cmd_nonzero and not fb_nonzero:
            self._zero_ticks += 1
        else:
            self._zero_ticks = 0
            if self._using_cmd and fb_nonzero:
                self.get_logger().info('[odom] Feedback recovered — switching back to FB mode.')
                self._using_cmd = False
                self._warn_printed = False

        if self._zero_ticks >= ZERO_TICKS_THRESHOLD:
            self.get_logger().warn('[odom] CMD fallback active', throttle_duration_sec=5.0)
            self._warn_printed = True
            self._using_cmd = True

        # ── Compute wheel velocities ──────────────────────────────
        if self._using_cmd:
            # CMD fallback: left_cmd/right_cmd sont déjà en RPM absolus
            # Le signe est intégré dans l_sign côté bridge → ne pas le réappliquer
            vl = rpm_to_mps(abs(self.left_cmd))
            vr = rpm_to_mps(abs(self.right_cmd))
        else:
            rl = self.left_vel  * self.l_sign
            rr = self.right_vel * self.r_sign
            # Auto-detect: rad/s (<50) vs raw DXL units (>=50)
            if abs(rl) < 50.0 and abs(rr) < 50.0:
                vl = rl * self.wheel_r
                vr = rr * self.wheel_r
            else:
                vl = rpm_to_mps(rl * DXL_VEL_TO_RPM)
                vr = rpm_to_mps(rr * DXL_VEL_TO_RPM)

        # ── Differential drive kinematics ─────────────────────────
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.wheel_b

        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        if self.wheel_r > 0:
            self.left_pos  += (vl / self.wheel_r) * dt
            self.right_pos += (vr / self.wheel_r) * dt

        q     = yaw_to_quat(self.yaw)
        stamp = now.to_msg()

        # ── /odom ─────────────────────────────────────────────────
        o = Odometry()
        o.header.stamp          = stamp
        o.header.frame_id       = 'odom'
        o.child_frame_id        = 'base_link'
        o.pose.pose.position.x  = self.x
        o.pose.pose.position.y  = self.y
        o.pose.pose.orientation = q
        o.pose.covariance       = POSE_COV
        o.twist.twist.linear.x  = v
        o.twist.twist.angular.z = w
        o.twist.covariance      = TWIST_COV
        self.odom_pub.publish(o)

        # ── odom → base_link TF ───────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp             = stamp
        tf.header.frame_id          = 'odom'
        tf.child_frame_id           = 'base_link'
        tf.transform.translation.x  = self.x
        tf.transform.translation.y  = self.y
        tf.transform.translation.z  = 0.0
        tf.transform.rotation       = q
        self.tf_br.sendTransform(tf)

        # ── /joint_states (wheel animation) ──────────────────────
        js = JointState()
        js.header.stamp = stamp
        js.name     = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos,  self.right_pos]
        js.velocity = [vl / self.wheel_r if self.wheel_r > 0 else 0.0,
                       vr / self.wheel_r if self.wheel_r > 0 else 0.0]
        js.effort   = [0.0, 0.0]
        self.js_pub.publish(js)

        src = 'CMD' if self._using_cmd else 'FB'
        self.get_logger().info(
            f'[DBG|{src}] L={self.left_vel:+.1f} R={self.right_vel:+.1f} raw | '
            f'vl={vl:+.4f} vr={vr:+.4f} m/s | '
            f'x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}°',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

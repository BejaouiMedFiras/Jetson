#!/usr/bin/env python3
"""
diff_drive_controller.py v3.1  —  PID + LIVE DIAG
═══════════════════════════════════════════════════
FIXES v3.1 vs v3.0:
  [FIX 1] LEFT_TRIM = 0.40
      Left wheel was spinning 2.5× faster than right (measured from encoder
      data: L=+1171°  R=−460° with identical RPM commands).
      Reducing LEFT_TRIM to 0.40 equalises wheel speeds.
      Tune in steps of ±0.05 until the robot tracks straight.

  [FIX 2] profile_acceleration = 300.0 rpm/s² sent at init
      ESP32 default ramp = 20 rpm/s → needs 12.5 s to reach 250 RPM.
      Watchdog fires after 5 s → motors always stopped before target speed.
      300 rpm/s² → full speed in < 1 s.

  [FIX 3] TIMEOUT raised from 5 s → 15 s
      Gives plenty of time for the ramp to complete before watchdog.

  [FIX 4] profile_acceleration re-sent on every torque re-enable
      After a watchdog stop the ESP32 resets its ramp to default (20 rpm/s).
      We re-send 300 rpm/s² every time we re-enable torque.

  [FIX 5] MIN_FF_RPM raised from 32 → 50
      With slew limiting on the ESP32 (PWM_SLEW_RATE = 80/s) a 32 RPM
      feedforward wasn't enough to overcome static friction. 50 RPM gives
      a larger initial kick.
"""

import math
import time
import threading
import collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from dynamixel_interfaces.msg import DynamixelCommand, DynamixelState
from std_srvs.srv import SetBool


# ══════════════════════════════════════════════════════════════════════════════
# ROBOT GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════
WHEEL_RADIUS   = 0.033    # metres
WHEEL_BASE     = 0.230    # metres

LEFT_SERVO_ID  = 1
RIGHT_SERVO_ID = 2

LEFT_SIGN      = +1.0     # +1 = positive RPM → wheel forward
RIGHT_SIGN     = -1.0     # -1 = right wheel physically mirrored

# [FIX 1] LEFT was 2.5× faster than RIGHT → reduce to 0.40
# Tune in ±0.05 steps until robot goes perfectly straight.
LEFT_TRIM      = 1.00
RIGHT_TRIM     = 1.00

MAX_RPM        = 60.0
VEL_SCALE      = 1.0

# [FIX 5] Raised from 32 → 50 RPM (overcomes static friction with slew limiter)
MIN_FF_RPM     = 40.0

# ── PID gains — straight driving ─────────────────────────────────────────────
Kp_straight    = 1.50
Ki_straight    = 0.50
Kd_straight    = 0.01

# ── PID gains — turning ──────────────────────────────────────────────────────
Kp_turn        = 1.00
Ki_turn        = 0.25
Kd_turn        = 0.00

TURN_THRESHOLD = 0.15     # rad/s — above this we switch to turn gains

PID_OUT_MAX    = 15.0
PID_OUT_MIN    = -15.0
INTEGRAL_MAX   = 15.0
INTEGRAL_MIN   = -15.0

# [FIX 3] Raised from 5 s → 15 s so ramp can complete
TIMEOUT        = 15.0
CONTROL_HZ     = 20.0
DIAG_HZ        = 1.0

# [FIX 2] Acceleration override sent to ESP32 (overrides DEFAULT_ACCEL_RPM_S=20)
PROFILE_ACCEL_RPM_S2 = 12000.0

# ── Auto-diagnosis thresholds ─────────────────────────────────────────────────
SLIDE_RATIO     = 0.80
SLIDE_TIME      = 1.5
OVERSHOOT_RATIO = 1.20
OSCILL_WINDOW   = 20
OSCILL_CROSS    = 6
STEADY_ERR_THR  = 8.0
STEADY_ERR_TIME = 2.0


# ══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def mps_to_rpm(mps):
    return (mps / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0 * VEL_SCALE

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _sign_changes(seq):
    n = 0
    for i in range(1, len(seq)):
        if seq[i-1] * seq[i] < 0:
            n += 1
    return n


# ══════════════════════════════════════════════════════════════════════════════
# VELOCITY PID
# ══════════════════════════════════════════════════════════════════════════════
class VelocityPID:
    def __init__(self, name):
        self.name       = name
        self.integral   = 0.0
        self.derivative = 0.0
        self.last_error = 0.0
        self._prev_meas = None

    def reset(self):
        self.integral   = 0.0
        self.derivative = 0.0
        self.last_error = 0.0
        self._prev_meas = None

    def update(self, setpoint, measured, dt, is_turning):
        if dt <= 0.0:
            return 0.0
        kp = Kp_turn if is_turning else Kp_straight
        ki = Ki_turn if is_turning else Ki_straight
        kd = Kd_turn if is_turning else Kd_straight

        error = setpoint - measured
        self.last_error = error

        if abs(setpoint) > 1.0:
            self.integral = _clamp(
                self.integral + error * dt,
                INTEGRAL_MIN / max(ki, 1e-9),
                INTEGRAL_MAX / max(ki, 1e-9),
            )
        else:
            self.integral *= 0.90

        if self._prev_meas is None:
            self.derivative = 0.0
        else:
            self.derivative = -(measured - self._prev_meas) / dt
        self._prev_meas = measured

        return _clamp(kp*error + ki*self.integral + kd*self.derivative,
                      PID_OUT_MIN, PID_OUT_MAX)


# ══════════════════════════════════════════════════════════════════════════════
# WHEEL DIAGNOSTICS
# ══════════════════════════════════════════════════════════════════════════════
class WheelDiag:
    def __init__(self, name):
        self.name         = name
        self._buf         = collections.deque(maxlen=300)
        self._slide_t     = None
        self._over_t      = None
        self._steady_t    = None
        self.diagnosis    = 'WAITING'
        self.suggestion   = ''

    def record(self, sp, measured):
        if abs(sp) < 5.0:
            self._slide_t = self._over_t = self._steady_t = None
            self.diagnosis  = 'IDLE'
            self.suggestion = ''
            return
        self._buf.append((time.monotonic(), sp - measured, sp))
        self._analyse()

    def _analyse(self):
        if len(self._buf) < 5:
            self.diagnosis = 'WAITING'
            return
        now    = time.monotonic()
        recent = [(t, e, s) for t, e, s in self._buf if now - t <= SLIDE_TIME]
        if not recent:
            return
        errs = [e for _, e, _ in recent]
        sps  = [s for _, _, s in recent]
        meas = [s - e for s, e in zip(sps, errs)]
        avg_sp   = sum(sps)  / len(sps)
        avg_meas = sum(meas) / len(meas)
        avg_err  = sum(errs) / len(errs)

        if len(errs) >= OSCILL_WINDOW:
            sc = _sign_changes(list(errs)[-OSCILL_WINDOW:])
            if sc >= OSCILL_CROSS:
                self.diagnosis  = '⚡ OSCILLATING'
                self.suggestion = (
                    f'Error sign flips {sc}x/s — Kp too high or Kd too low.\n'
                    f'   → Kp_straight -= 0.15   or   Kd_straight += 0.01'
                )
                return

        if avg_sp > 5.0 and avg_meas < SLIDE_RATIO * avg_sp:
            if self._slide_t is None:
                self._slide_t = now
            elif now - self._slide_t >= SLIDE_TIME:
                self.diagnosis  = '🐢 SLIDING (too slow)'
                self.suggestion = (
                    f'meas {avg_meas:.0f} rpm < {SLIDE_RATIO*100:.0f}% of SP {avg_sp:.0f} rpm '
                    f'for {now-self._slide_t:.1f}s.\n'
                    f'   → Kp_straight += 0.2   or   Ki_straight += 0.05\n'
                    f'   → MIN_FF_RPM currently {MIN_FF_RPM:.0f} — raise if stalling'
                )
                self._over_t = None
                return
        else:
            self._slide_t = None

        if avg_sp > 5.0 and avg_meas > OVERSHOOT_RATIO * avg_sp:
            if self._over_t is None:
                self._over_t = now
            elif now - self._over_t >= SLIDE_TIME:
                self.diagnosis  = '🚀 OVERSHOOT (too fast)'
                self.suggestion = (
                    f'meas {avg_meas:.0f} rpm > {OVERSHOOT_RATIO*100:.0f}% of SP {avg_sp:.0f} rpm.\n'
                    f'   → Kp_straight -= 0.2   or   Kd_straight += 0.01\n'
                    f'   → VEL_SCALE may be too high'
                )
                self._slide_t = None
                return
        else:
            self._over_t = None

        if abs(avg_err) > STEADY_ERR_THR:
            if self._steady_t is None:
                self._steady_t = now
            elif now - self._steady_t >= STEADY_ERR_TIME:
                self.diagnosis  = '📏 STEADY ERROR'
                self.suggestion = (
                    f'Persistent error {avg_err:+.1f} rpm over {now-self._steady_t:.1f}s.\n'
                    f'   → Ki_straight += 0.05   (eliminate bias)\n'
                    f'   → Check INTEGRAL_MAX — may be clamping too early'
                )
                return
        else:
            self._steady_t = None

        self.diagnosis  = '✅ OK'
        self.suggestion = ''


# ══════════════════════════════════════════════════════════════════════════════
# COMMAND BUILDER
# ══════════════════════════════════════════════════════════════════════════════
def _make_vel_cmd(servo_id, rpm, profile_accel=float('nan')):
    """Build a DynamixelCommand for velocity control."""
    cmd = DynamixelCommand()
    cmd.servo_id             = servo_id
    cmd.operating_mode       = -1            # keep current mode
    cmd.torque_enable        = -1            # keep current torque
    cmd.goal_velocity        = float(rpm)
    cmd.goal_position        = float('nan')
    cmd.goal_current         = float('nan')
    cmd.goal_pwm             = float('nan')
    cmd.profile_velocity     = float('nan')
    cmd.profile_acceleration = profile_accel
    return cmd


def _make_init_cmd(servo_id):
    """Build the one-time init command: velocity mode + fast ramp."""
    cmd = DynamixelCommand()
    cmd.servo_id             = servo_id
    cmd.operating_mode       = 1             # VELOCITY mode
    cmd.torque_enable        = 0             # torque OFF during mode change
    cmd.goal_velocity        = 0.0
    cmd.goal_position        = float('nan')
    cmd.goal_current         = float('nan')
    cmd.goal_pwm             = float('nan')
    cmd.profile_velocity     = float('nan')
    # [FIX 2] Override ESP32 default ramp (20 rpm/s) with 300 rpm/s²
    cmd.profile_acceleration = PROFILE_ACCEL_RPM_S2
    return cmd


# ══════════════════════════════════════════════════════════════════════════════
# MAIN NODE
# ══════════════════════════════════════════════════════════════════════════════
class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        self._sp_left    = 0.0
        self._sp_right   = 0.0
        self._is_turning = False

        self._meas_left  = 0.0
        self._meas_right = 0.0
        self._enc_left   = 0.0
        self._enc_right  = 0.0
        self._last_state_left  = 0.0
        self._last_state_right = 0.0

        self._pid_left  = VelocityPID('left')
        self._pid_right = VelocityPID('right')

        self._cmd_left_servo  = 0.0
        self._cmd_right_servo = 0.0
        self._corr_left       = 0.0
        self._corr_right      = 0.0

        self._diag_left  = WheelDiag('LEFT')
        self._diag_right = WheelDiag('RIGHT')

        self._lock           = threading.Lock()
        self._initialized    = False
        self._motors_running = False
        self._last_cmd_time  = self.get_clock().now()
        self._start_time     = time.monotonic()

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_left  = self.create_publisher(
            DynamixelCommand, f'/servo{LEFT_SERVO_ID}/command',  10)
        self.pub_right = self.create_publisher(
            DynamixelCommand, f'/servo{RIGHT_SERVO_ID}/command', 10)

        self.torque_left  = self.create_client(SetBool, f'/servo{LEFT_SERVO_ID}/set_torque')
        self.torque_right = self.create_client(SetBool, f'/servo{RIGHT_SERVO_ID}/set_torque')

        self.create_subscription(Twist, '/cmd_vel',        self._cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._cmd_vel_cb, 10)
        self.create_subscription(DynamixelState, f'/servo{LEFT_SERVO_ID}/state',
                                 self._state_left_cb,  qos_state)
        self.create_subscription(DynamixelState, f'/servo{RIGHT_SERVO_ID}/state',
                                 self._state_right_cb, qos_state)

        self.create_timer(1.0,               self._init_motors)
        self.create_timer(0.1,               self._watchdog)
        self.create_timer(1.0 / CONTROL_HZ,  self._pid_loop)
        self.create_timer(1.0 / DIAG_HZ,     self._print_diagnostic)

        print(
            '\n\033[1m\033[36m'
            '╔══════════════════════════════════════════════════════╗\n'
            '║  diff_drive_controller v3.1  —  PID + LIVE DIAG     ║\n'
            f'║  Kp={Kp_straight}  Ki={Ki_straight}  Kd={Kd_straight}'
            f'  MIN_FF={MIN_FF_RPM:.0f} rpm'
            '                    ║\n'
            f'║  L_TRIM={LEFT_TRIM:.2f}  R_TRIM={RIGHT_TRIM:.2f}'
            f'  ACCEL={PROFILE_ACCEL_RPM_S2:.0f} rpm/s²'
            '              ║\n'
            '╚══════════════════════════════════════════════════════╝'
            '\033[0m', flush=True
        )

    # ── Encoder feedback ──────────────────────────────────────────────────────

    def _state_left_cb(self, msg: DynamixelState):
        with self._lock:
            self._meas_left         = msg.present_velocity * LEFT_SIGN
            self._enc_left          = msg.present_position
            self._last_state_left   = time.monotonic()

    def _state_right_cb(self, msg: DynamixelState):
        with self._lock:
            self._meas_right        = msg.present_velocity * RIGHT_SIGN
            self._enc_right         = msg.present_position
            self._last_state_right  = time.monotonic()

    # ── cmd_vel → setpoints ───────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        if not self._initialized:
            return
        self._last_cmd_time = self.get_clock().now()
        v, w = msg.linear.x, msg.angular.z
        # [FIX 1] Apply LEFT_TRIM=0.40 to balance wheels
        sp_l = _clamp(mps_to_rpm(v - w*WHEEL_BASE/2.0) * LEFT_TRIM,  -MAX_RPM, MAX_RPM)
        sp_r = _clamp(mps_to_rpm(v + w*WHEEL_BASE/2.0) * RIGHT_TRIM, -MAX_RPM, MAX_RPM)
        with self._lock:
            self._sp_left    = sp_l
            self._sp_right   = sp_r
            self._is_turning = abs(w) >= TURN_THRESHOLD
        if not self._motors_running and (sp_l != 0.0 or sp_r != 0.0):
            self._enable_torque_with_ramp()
            self._motors_running = True

    # ── PID loop ──────────────────────────────────────────────────────────────

    def _pid_loop(self):
        if not self._initialized or not self._motors_running:
            return
        dt = 1.0 / CONTROL_HZ
        with self._lock:
            sp_l, sp_r     = self._sp_left,  self._sp_right
            ml, mr         = self._meas_left, self._meas_right
            is_turning     = self._is_turning

        corr_l = self._pid_left.update(sp_l,  ml, dt, is_turning)
        corr_r = self._pid_right.update(sp_r, mr, dt, is_turning)

        def resolve(sp, corr, pid):
            if abs(sp) < 1.0:
                pid.reset()
                return 0.0
            ff = sp
            if 0.0 < abs(ff) < MIN_FF_RPM:
                ff = math.copysign(MIN_FF_RPM, ff)
            return _clamp(ff + corr, -MAX_RPM * 3, MAX_RPM * 3)

        cmd_l = resolve(sp_l, corr_l, self._pid_left)
        cmd_r = resolve(sp_r, corr_r, self._pid_right)

        cmd_ls = cmd_l * LEFT_SIGN
        cmd_rs = cmd_r * RIGHT_SIGN

        with self._lock:
            self._cmd_left_servo  = cmd_ls
            self._cmd_right_servo = cmd_rs
            self._corr_left       = corr_l
            self._corr_right      = corr_r

        self._diag_left.record(sp_l,  ml)
        self._diag_right.record(sp_r, mr)

        # Send velocity commands (no profile_accel needed every tick)
        self.pub_left.publish(_make_vel_cmd(LEFT_SERVO_ID,   cmd_ls))
        self.pub_right.publish(_make_vel_cmd(RIGHT_SERVO_ID, cmd_rs))

    # ── Motor init ────────────────────────────────────────────────────────────

    def _init_motors(self):
        if self._initialized:
            return
        if not self.torque_left.wait_for_service(timeout_sec=0.1):
            return
        if not self.torque_right.wait_for_service(timeout_sec=0.1):
            return

        self.get_logger().info('Setting VELOCITY mode + 300 rpm/s² ramp on S1 + S2 ...')

        # [FIX 2] Send init command with fast profile_acceleration
        for sid, pub in [(LEFT_SERVO_ID,  self.pub_left),
                         (RIGHT_SERVO_ID, self.pub_right)]:
            pub.publish(_make_init_cmd(sid))

        time.sleep(0.5)   # allow ESP32 to apply mode + ramp setting

        self._enable_torque_with_ramp()
        self._initialized    = True
        self._motors_running = True
        self.get_logger().info(
            f'Both motors ready — L_TRIM={LEFT_TRIM} R_TRIM={RIGHT_TRIM} '
            f'ACCEL={PROFILE_ACCEL_RPM_S2} rpm/s²'
        )

    def _enable_torque_with_ramp(self):
        """Enable torque AND re-send profile_acceleration (ESP32 resets on torque-off)."""
        # [FIX 4] Re-send profile_acceleration before enabling torque
        for sid, pub in [(LEFT_SERVO_ID,  self.pub_left),
                         (RIGHT_SERVO_ID, self.pub_right)]:
            cmd = DynamixelCommand()
            cmd.servo_id             = sid
            cmd.operating_mode       = -1
            cmd.torque_enable        = -1
            cmd.goal_velocity        = 0.0
            cmd.goal_position        = float('nan')
            cmd.goal_current         = float('nan')
            cmd.goal_pwm             = float('nan')
            cmd.profile_velocity     = float('nan')
            cmd.profile_acceleration = PROFILE_ACCEL_RPM_S2
            pub.publish(cmd)

        time.sleep(0.05)

        for client in (self.torque_left, self.torque_right):
            req = SetBool.Request()
            req.data = True
            client.call_async(req)

    def _set_torque_both(self, enable):
        for client in (self.torque_left, self.torque_right):
            req = SetBool.Request()
            req.data = enable
            client.call_async(req)
        if not enable:
            self._pid_left.reset()
            self._pid_right.reset()

    def _watchdog(self):
        if not self._initialized:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        # [FIX 3] TIMEOUT raised to 15 s
        if elapsed > TIMEOUT and self._motors_running:
            self.get_logger().info('watchdog: no cmd_vel — stopping motors')
            with self._lock:
                self._sp_left = self._sp_right = 0.0
            self.pub_left.publish(_make_vel_cmd(LEFT_SERVO_ID,   0.0))
            self.pub_right.publish(_make_vel_cmd(RIGHT_SERVO_ID, 0.0))
            self._set_torque_both(False)
            self._motors_running = False

    # ── LIVE TERMINAL DIAGNOSTIC ──────────────────────────────────────────────

    def _print_diagnostic(self):
        now = time.monotonic()
        t   = now - self._start_time

        with self._lock:
            sp_l, sp_r   = self._sp_left,  self._sp_right
            ml, mr       = self._meas_left, self._meas_right
            enc_l, enc_r = self._enc_left,  self._enc_right
            cmd_l        = self._cmd_left_servo
            cmd_r        = self._cmd_right_servo
            corr_l       = self._corr_left
            corr_r       = self._corr_right

        err_l = sp_l - ml
        err_r = sp_r - mr
        pid_l = self._pid_left
        pid_r = self._pid_right

        stale_l = (now - self._last_state_left  > 0.5) if self._last_state_left  else True
        stale_r = (now - self._last_state_right > 0.5) if self._last_state_right else True

        mode_s = 'TURNING ' if self._is_turning else 'STRAIGHT'
        mtr_s  = 'ON'       if self._motors_running else 'OFF'
        kp = Kp_turn if self._is_turning else Kp_straight
        ki = Ki_turn if self._is_turning else Ki_straight
        kd = Kd_turn if self._is_turning else Kd_straight

        R  = '\033[0m';  B  = '\033[1m';  CY = '\033[36m'
        YL = '\033[33m'; RD = '\033[31m'; GN = '\033[32m'
        DM = '\033[2m';  MG = '\033[35m'

        def ce(e):
            if abs(e) < 5:  return GN
            if abs(e) < 20: return YL
            return RD

        def stale_tag(s):
            return f' {RD}{B}[NO DATA]{R}' if s else ''

        W = 67

        def pad(s, w):
            import re
            plain = re.sub(r'\033\[[0-9;]*m', '', s)
            return s + ' ' * max(0, w - len(plain))

        sep_top = f'{CY}┌{"─"*14}┬{"─"*(W-14)}┐{R}'
        sep_mid = f'{CY}├{"─"*14}┼{"─"*(W-14)}┤{R}'
        sep_low = f'{CY}├{"─"*14}┴{"─"*(W-14)}┤{R}'
        sep_bot = f'{CY}└{"─"*(W+1)}┘{R}'

        def row(left_col, right_col):
            lp = pad(left_col,  14)
            rp = pad(right_col, W-16)
            return f'{CY}│{R}{lp}{CY}│{R} {rp} {CY}│{R}'

        def full_row(content):
            cp = pad(content, W-1)
            return f'{CY}│{R} {cp}{CY}│{R}'

        hdr = (f'{B}DIFF DRIVE v3.1{R}  ▸  '
               f't={t:6.1f}s   mode={B}{mode_s}{R}   '
               f'motors={B}{GN if self._motors_running else RD}{mtr_s}{R}')

        lines = ['', sep_top, full_row(hdr), sep_mid]

        for side, sp, ml2, enc, cmd, corr, pid, stale in [
            ('LEFT  (S1)', sp_l, ml,  enc_l, cmd_l, corr_l, pid_l, stale_l),
            ('RIGHT (S2)', sp_r, mr,  enc_r, cmd_r, corr_r, pid_r, stale_r),
        ]:
            err = sp - ml2
            r1 = (f'SP={B}{sp:+7.1f}{R}  meas={B}{ml2:+7.1f}{R}  '
                  f'err={ce(err)}{B}{err:+7.1f}{R}  cmd={B}{cmd:+7.1f}{R} rpm'
                  f'{stale_tag(stale)}')
            r2 = (f'enc={B}{enc:+10.2f}°{R}  '
                  f'{DM}Kp={kp:.2f}  Ki={ki:.2f}  Kd={kd:.3f}{R}')
            r3 = (f'integral={MG}{pid.integral:+8.2f}{R}  '
                  f'deriv={MG}{pid.derivative:+8.2f}{R}  '
                  f'corr={MG}{corr:+6.1f}{R} rpm')
            lines.append(row(f' {B}{side}{R}  ', r1))
            lines.append(row('              ', r2))
            lines.append(row('              ', r3))
            if side == 'LEFT  (S1)':
                lines.append(sep_mid)

        lines.append(sep_low)

        diags = []
        for s, d in [('LEFT', self._diag_left), ('RIGHT', self._diag_right)]:
            if d.diagnosis not in ('✅ OK', 'IDLE', 'WAITING'):
                diags.append((s, d.diagnosis, d.suggestion))

        if diags:
            for s, diag, sug in diags:
                lines.append(full_row(f'{YL}{B}{s}  {diag}{R}'))
                for line in sug.split('\n'):
                    lines.append(full_row(f'  {DM}{line.strip()}{R}'))
        else:
            dl = self._diag_left.diagnosis
            dr = self._diag_right.diagnosis
            lines.append(full_row(
                f'{GN}✅  Both wheels OK{R}  {DM}(L: {dl}  R: {dr}){R}'
            ))

        lines.append(sep_bot)
        print('\n'.join(lines), flush=True)

    def destroy_node(self):
        try:
            self.pub_left.publish(_make_vel_cmd(LEFT_SERVO_ID,   0.0))
            self.pub_right.publish(_make_vel_cmd(RIGHT_SERVO_ID, 0.0))
            self._set_torque_both(False)
        except Exception:
            pass
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

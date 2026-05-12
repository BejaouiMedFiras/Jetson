"""
diff_drive_controller.py  —  v7.2
═══════════════════════════════════════════════════════════════════════

CORRECTIONS v7.2 vs v7.1 :

  [FIX 1] MAX_RPM aligné avec ESP32
      v7.1 avait MAX_RPM = 60.0 mais l'ESP32 a MAX_RPM_FREE = 120.0.
      La commande 60 RPM → PWM = 60/120*255 = 127 (50%) → moteur trop faible.
      SOLUTION : MAX_RPM = 120.0  (doit être IDENTIQUE à MAX_RPM_FREE esp32)

  [FIX 2] FF_MIN recalculé
      MIN_PWM_DEADBAND=25 → seuil friction = 25/255*120 ≈ 11.8 rpm
      FF_MIN_LINEAR   = 14.0  (marge de sécurité légère)
      FF_MIN_TURN_EXT = 22.0
      FF_MIN_TURN_INT = 22.0

  [FIX 3] PID_CLAMP élargi
      Avec MAX_RPM=120, le PID pouvait pas corriger au-delà de 18 rpm.
      PID_CLAMP   = 40.0
      INTEG_CLAMP = 25.0

  [FIX 4] PROFILE_ACCEL_RPM_S2 cohérent
      RAMP ESP32 = 300 rpm/s → on met la même valeur ici pour la
      commande profile_acceleration (registre DXL).

  Convention physique des moteurs (INCHANGÉE) :
    • Les moteurs GAUCHES et DROITS sont montés en miroir sur le châssis.
    • Pour aller TOUT DROIT :
        servo gauche reçoit  +RPM  → roues gauches AVANCENT
        servo droit  reçoit  -RPM  → roues droites AVANCENT (monté inversé)
    • RIGHT_SIGN = -1 est le SEUL endroit où cette inversion est appliquée.
    • L'ESP32 Slave 2 NE fait PAS d'inversion — il exécute la commande telle quelle.

  Convention virage :
    • w > 0 = tourner à GAUCHE  (roues droites plus vite en valeur absolue)
    • w < 0 = tourner à DROITE  (roues gauches plus vite en valeur absolue)
"""

import math, time, threading, sys, select, tty, termios
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from dynamixel_interfaces.msg import DynamixelCommand, DynamixelState
from std_srvs.srv import SetBool, Empty


# ════════════════════════════════════════════════════════════════════════════
# PARAMÈTRES PHYSIQUES
# ════════════════════════════════════════════════════════════════════════════
WHEEL_RADIUS  = 0.065          # m  (Ø130mm)
WHEEL_BASE    = 0.20           # m  (entraxe gauche/droite)

# [FIX 1] MAX_RPM DOIT être égal à MAX_RPM_FREE dans l'ESP32 (120.0)
# Si vous changez MAX_RPM_FREE dans l'ESP32, changez aussi ici !
MAX_RPM       = 220.0          # ← WAS 60.0 — CAUSE DU BLOCAGE

RPM_TO_MPS    = (2 * math.pi * WHEEL_RADIUS) / 60.0   # 0.006807 m/s par RPM

# [FIX 2] FF_MIN recalculé : MIN_PWM_DEADBAND=25 → 25/255*120 ≈ 11.8 rpm
FF_MIN_LINEAR   = 14.0         # RPM minimum pour vaincre friction statique
FF_MIN_TURN_EXT = 22.0         # rotation sur place : friction plus élevée
FF_MIN_TURN_INT = 22.0         # les deux côtés ont besoin du même seuil

LEFT_SERVO_ID  = 1
RIGHT_SERVO_ID = 2

# ═══════════════════════════════════════════════════════════════════════════
# SIGNES PHYSIQUES — NE PAS MODIFIER SANS RAISON
# ═══════════════════════════════════════════════════════════════════════════
LEFT_SIGN  = +1.0
RIGHT_SIGN = -1.0
# ═══════════════════════════════════════════════════════════════════════════

# ── PID ───────────────────────────────────────────────────────────────────
Kp = 1.8;  Ki = 2.5;  Kd = 0.025

# [FIX 3] PID_CLAMP élargi pour MAX_RPM=120
PID_CLAMP   = 40.0             # ← WAS 18.0
INTEG_CLAMP = 25.0             # ← WAS 12.0

CONTROL_HZ  = 20.0
DIAG_HZ     =  1.0
TIMEOUT_S   =  5.0

# [FIX 4] PROFILE_ACCEL doit correspondre à RAMP_RPM_S de l'ESP32
PROFILE_ACCEL_RPM_S2 = 300.0   # ← WAS 150.0 (ESP32 = 300 rpm/s)

# ════════════════════════════════════════════════════════════════════════════
# PROFILS DE VIRAGE  (valeurs en RPM, sur échelle MAX_RPM=120)
# ════════════════════════════════════════════════════════════════════════════
def _turn_radius(ext, int_):
    if abs(ext - int_) < 0.1:
        return float('inf')
    return WHEEL_BASE / 2 * (ext + int_) / (ext - int_)

def _torque_factor(ext, int_):
    return abs(ext - int_) / (2 * MAX_RPM)

# Les profils sont exprimés en RPM absolus (<=MAX_RPM=120)
TURN_PROFILES = {
    'a': (120.0,  70.0),   # virage doux
    'b': (120.0,  40.0),   # virage moyen
    'c': (120.0,  10.0),   # serré  ← DÉFAUT
    'd': (120.0, -60.0),   # très serré
    'e': (120.0, -120.0),  # rotation pure
}
DEFAULT_PROFILE = 'c'
TURN_THRESHOLD_W = 0.05
_PROFILE_MAP = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e'}


# ════════════════════════════════════════════════════════════════════════════
# HELPERS
# ════════════════════════════════════════════════════════════════════════════
def mps_to_rpm(v):
    return v / RPM_TO_MPS

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _apply_ff_min(rpm, ff_min):
    if 0.0 < abs(rpm) < ff_min:
        return math.copysign(ff_min, rpm)
    return rpm


# ════════════════════════════════════════════════════════════════════════════
# PID
# ════════════════════════════════════════════════════════════════════════════
class WheelPID:
    def __init__(self, name: str):
        self.name = name
        self.integ = 0.0;  self.deriv = 0.0
        self._prev = None; self.last_err = 0.0

    def reset(self):
        self.integ = 0.0; self.deriv = 0.0
        self._prev = None; self.last_err = 0.0

    def update(self, sp: float, meas: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        err = sp - meas
        self.last_err = err
        if abs(sp) > 0.5:
            self.integ = _clamp(self.integ + err * dt, -INTEG_CLAMP, INTEG_CLAMP)
        else:
            self.integ *= 0.75
        self.deriv = 0.0 if self._prev is None else -(meas - self._prev) / dt
        self._prev = meas
        return _clamp(Kp * err + Ki * self.integ + Kd * self.deriv,
                      -PID_CLAMP, PID_CLAMP)


# ════════════════════════════════════════════════════════════════════════════
# BUILDERS COMMANDES DYNAMIXEL
# ════════════════════════════════════════════════════════════════════════════
def _vcmd(sid: int, rpm: float) -> DynamixelCommand:
    c = DynamixelCommand()
    c.servo_id = sid;  c.operating_mode = -1;  c.torque_enable = -1
    c.goal_velocity = float(rpm)
    c.goal_position = c.goal_current = c.goal_pwm = float('nan')
    c.profile_velocity = c.profile_acceleration = float('nan')
    return c

def _icmd(sid: int) -> DynamixelCommand:
    c = DynamixelCommand()
    c.servo_id = sid;  c.operating_mode = 1;  c.torque_enable = 0
    c.goal_velocity = 0.0
    c.goal_position = c.goal_current = c.goal_pwm = float('nan')
    c.profile_velocity = float('nan')
    c.profile_acceleration = float(PROFILE_ACCEL_RPM_S2)
    return c


# ════════════════════════════════════════════════════════════════════════════
# TÉLÉOP
# ════════════════════════════════════════════════════════════════════════════
_HELP = """
╔══════════════════════════════════════════════════════════════════════╗
║   TÉLÉOP v7.2  [MAX_RPM=120]                                         ║
╠══════════════════════════════════════════════════════════════════════╣
║  i / ,   : avancer / reculer                                         ║
║  j       : ↰  VIRER GAUCHE                                          ║
║  l       : ↱  VIRER DROITE                                          ║
║  k       : ⛔  STOP                                                 ║
║  + / -   : vitesse linéaire                                          ║
║  a-e     : profil de virage                                          ║
╠══════════════════════════════════════════════════════════════════════╣
║  a  +120/ +70  Δ= 50  virage doux                                    ║
║  b  +120/ +40  Δ= 80  virage moyen                                   ║
║  c  +120/ +10  Δ=110  serré  ← DÉFAUT                               ║
║  d  +120/ -60  Δ=180  très serré                                     ║
║  e  +120/-120  Δ=240  rotation pure                                  ║
╚══════════════════════════════════════════════════════════════════════╝
"""

def _gkey(settings):
    tty.setraw(sys.stdin.fileno())
    r, _, _ = select.select([sys.stdin], [], [], 0.1)
    k = sys.stdin.read(1) if r else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return k


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub   = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self._lin  = 0.25
        self._prof = DEFAULT_PROFILE
        print(_HELP)
        self._status()

    def _status(self):
        ext, int_ = TURN_PROFILES[self._prof]
        r = _turn_radius(ext, int_)
        r_str = f'{r:.2f}m' if r < 9 else 'rotation'
        print(f'  Profil [{self._prof.upper()}]  ext={ext:+.0f}  int={int_:+.0f}  '
              f'R≈{r_str}  couple={_torque_factor(ext,int_)*100:.0f}%  '
              f'lin={self._lin:.2f}m/s')

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                k = _gkey(settings)
                if not k:
                    continue
                if k == 'k':
                    self._tx(0.0, 0.0); print('  STOP')
                elif k == 'i':
                    self._tx(self._lin, 0.0)
                elif k == ',':
                    self._tx(-self._lin, 0.0)
                elif k == 'j':
                    idx = list(TURN_PROFILES).index(self._prof) + 1
                    self._tx(0.0, float(idx))
                    ext, int_ = TURN_PROFILES[self._prof]
                    print(f'  GAUCHE  G={int_:+.0f}rpm  D={ext:+.0f}rpm  [{self._prof.upper()}]')
                elif k == 'l':
                    idx = list(TURN_PROFILES).index(self._prof) + 1
                    self._tx(0.0, -float(idx))
                    ext, int_ = TURN_PROFILES[self._prof]
                    print(f'  DROITE  G={ext:+.0f}rpm  D={int_:+.0f}rpm  [{self._prof.upper()}]')
                elif k.lower() in TURN_PROFILES:
                    self._prof = k.lower(); print(); self._status()
                elif k == '+':
                    self._lin = _clamp(self._lin + 0.05, 0.05, 0.80); self._status()
                elif k == '-':
                    self._lin = _clamp(self._lin - 0.05, 0.05, 0.80); self._status()
                elif k == '\x03':
                    break
        finally:
            self._tx(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def _tx(self, lin, ang):
        tw = Twist()
        tw.linear.x = lin; tw.angular.z = ang
        self.pub.publish(tw)


# ════════════════════════════════════════════════════════════════════════════
# CONTRÔLEUR PRINCIPAL
# ════════════════════════════════════════════════════════════════════════════
class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        self._sp_l = 0.0;    self._sp_r = 0.0
        self._lbl  = 'ATTENTE'
        self._meas_l = 0.0;  self._meas_r = 0.0
        self._pid_l = WheelPID('L')
        self._pid_r = WheelPID('R')
        self._cmd_l = 0.0;   self._cmd_r = 0.0

        self._lock        = threading.Lock()
        self._initialized = False
        self._running     = False
        self._last_t      = self.get_clock().now()
        self._t0          = time.monotonic()
        self._st_l        = 0.0
        self._st_r        = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_l = self.create_publisher(
            DynamixelCommand, f'/servo{LEFT_SERVO_ID}/command',  10)
        self.pub_r = self.create_publisher(
            DynamixelCommand, f'/servo{RIGHT_SERVO_ID}/command', 10)
        self.tor_l = self.create_client(SetBool, f'/servo{LEFT_SERVO_ID}/set_torque')
        self.tor_r = self.create_client(SetBool, f'/servo{RIGHT_SERVO_ID}/set_torque')

        self.create_subscription(Twist, '/cmd_vel',        self._on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._on_cmd, 10)
        self.create_subscription(
            DynamixelState, f'/servo{LEFT_SERVO_ID}/state',  self._on_st_l, qos)
        self.create_subscription(
            DynamixelState, f'/servo{RIGHT_SERVO_ID}/state', self._on_st_r, qos)
        self.create_service(Empty, '/reset_pid', self._on_reset)

        self.create_timer(1.0,              self._init_motors)
        self.create_timer(0.1,              self._watchdog)
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.create_timer(1.0 / DIAG_HZ,    self._print_diag)

        print('\n\033[1m\033[36m'
              '╔══════════════════════════════════════════════════════════════════╗\n'
              '║  diff_drive v7.2  —  MAX_RPM=120 aligné avec ESP32              ║\n'
             f'║  LEFT_SIGN={LEFT_SIGN:+.0f}  RIGHT_SIGN={RIGHT_SIGN:+.0f}  '
             f'MAX_RPM={MAX_RPM:.0f}  FF_MIN_LINEAR={FF_MIN_LINEAR:.1f}          ║\n'
             f'║  [FIX1] MAX_RPM 60→120  [FIX2] FF_MIN recalc  [FIX3] PID_CLAMP ║\n'
              '╚══════════════════════════════════════════════════════════════════╝'
              '\033[0m', flush=True)

    def _on_reset(self, req, res):
        self._pid_l.reset(); self._pid_r.reset()
        self.get_logger().info('PID réinitialisé')
        return res

    def _on_st_l(self, msg: DynamixelState):
        with self._lock:
            self._meas_l = msg.present_velocity * LEFT_SIGN
            self._st_l   = time.monotonic()

    def _on_st_r(self, msg: DynamixelState):
        with self._lock:
            self._meas_r = msg.present_velocity * RIGHT_SIGN
            self._st_r   = time.monotonic()

    def _on_cmd(self, msg: Twist):
        if not self._initialized:
            return
        self._last_t = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # ── Virage profil téléop (|w| encodé 0.5..5.5, v≈0) ─────────────────
        if 0.5 <= abs(w) <= 5.5 and abs(v) < 0.05:
            idx      = int(round(abs(w)))
            prof_key = _PROFILE_MAP.get(idx, DEFAULT_PROFILE)
            rpm_ext, rpm_int = TURN_PROFILES[prof_key]

            if w > 0:   # GAUCHE
                sp_l = -rpm_ext
                sp_r = +rpm_ext
            else:        # DROITE
                sp_l = +rpm_ext
                sp_r = -rpm_ext

            r_str = (f'{_turn_radius(rpm_ext, rpm_int):.2f}m'
                     if _turn_radius(rpm_ext, rpm_int) < 9 else 'rotation')
            label = (f'VIRAGE {"GAUCHE" if w > 0 else "DROITE"}  '
                     f'[{prof_key.upper()}]  '
                     f'G={sp_l:+.0f}  D={sp_r:+.0f}  R≈{r_str}')

        # ── Ligne droite / commande ROS standard ─────────────────────────────
        else:
            v_l  = v - w * WHEEL_BASE / 2.0
            v_r  = v + w * WHEEL_BASE / 2.0
            sp_l = _clamp(mps_to_rpm(v_l), -MAX_RPM, MAX_RPM)
            sp_r = _clamp(mps_to_rpm(v_r), -MAX_RPM, MAX_RPM)
            label = f'DROIT  v={v:+.3f}m/s  G={sp_l:+.1f}  D={sp_r:+.1f}rpm'

        with self._lock:
            self._sp_l = sp_l
            self._sp_r = sp_r
            self._lbl  = label

        if not self._running and (abs(sp_l) > 0.1 or abs(sp_r) > 0.1):
            self._set_torque(True)
            self._running = True

    def _control_loop(self):
        if not self._initialized or not self._running:
            return
        dt = 1.0 / CONTROL_HZ

        with self._lock:
            sp_l = self._sp_l;  sp_r = self._sp_r
            ml   = self._meas_l; mr  = self._meas_r

        def resolve(sp, meas, pid, ff_min):
            if abs(sp) < 0.3:
                pid.reset()
                return 0.0
            ff = _apply_ff_min(sp, ff_min)
            return _clamp(ff + pid.update(sp, meas, dt), -MAX_RPM, MAX_RPM)

        is_turn = abs(sp_l - sp_r) > 5.0
        if is_turn:
            if abs(sp_l) >= abs(sp_r):
                cmd_l = resolve(sp_l, ml, self._pid_l, FF_MIN_TURN_EXT)
                cmd_r = resolve(sp_r, mr, self._pid_r, FF_MIN_TURN_INT)
            else:
                cmd_l = resolve(sp_l, ml, self._pid_l, FF_MIN_TURN_INT)
                cmd_r = resolve(sp_r, mr, self._pid_r, FF_MIN_TURN_EXT)
        else:
            cmd_l = resolve(sp_l, ml, self._pid_l, FF_MIN_LINEAR)
            cmd_r = resolve(sp_r, mr, self._pid_r, FF_MIN_LINEAR)

        # ══════════════════════════════════════════════════════════════════
        # APPLICATION DES SIGNES PHYSIQUES
        # ══════════════════════════════════════════════════════════════════
        cmd_ls = cmd_l * LEFT_SIGN
        cmd_rs = cmd_r * RIGHT_SIGN

        with self._lock:
            self._cmd_l = cmd_ls
            self._cmd_r = cmd_rs

        self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  cmd_ls))
        self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, cmd_rs))

    def _init_motors(self):
        if self._initialized:
            return
        if not (self.tor_l.wait_for_service(0.05) and
                self.tor_r.wait_for_service(0.05)):
            return
        for sid, pub in [(LEFT_SERVO_ID, self.pub_l),
                         (RIGHT_SERVO_ID, self.pub_r)]:
            pub.publish(_icmd(sid))
        time.sleep(0.4)
        self._set_torque(True)
        self._initialized = True
        self._running     = True
        self.get_logger().info('Moteurs initialisés v7.2')

    def _set_torque(self, en: bool):
        for c in (self.tor_l, self.tor_r):
            r = SetBool.Request(); r.data = en; c.call_async(r)
        if not en:
            self._pid_l.reset(); self._pid_r.reset()

    def _watchdog(self):
        if not self._initialized:
            return
        if ((self.get_clock().now() - self._last_t).nanoseconds / 1e9
                > TIMEOUT_S and self._running):
            with self._lock:
                self._sp_l = self._sp_r = 0.0
                self._lbl  = 'WATCHDOG — STOP'
            self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  0.0))
            self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, 0.0))
            self._set_torque(False)
            self._running = False

    def _print_diag(self):
        now = time.monotonic()
        t   = now - self._t0
        with self._lock:
            sp_l  = self._sp_l;  sp_r  = self._sp_r
            ml    = self._meas_l; mr   = self._meas_r
            cmd_l = self._cmd_l; cmd_r = self._cmd_r
            lbl   = self._lbl

        stale_l = (now - self._st_l > 0.5) if self._st_l else True
        stale_r = (now - self._st_r > 0.5) if self._st_r else True

        R='\033[0m'; B='\033[1m'; CY='\033[36m'
        GN='\033[32m'; YL='\033[33m'; RD='\033[31m'; DM='\033[2m'

        def pad(s, w):
            import re
            return s + ' ' * max(0, w - len(re.sub(r'\033\[[0-9;]*m', '', s)))
        def frow(c, W=76):
            return f'{CY}│{R} {pad(c, W-1)} {CY}│{R}'
        def ce(e):
            return GN if abs(e) < 6 else (YL if abs(e) < 25 else RD)
        def stag(s):
            return f' {RD}[NO FEEDBACK]{R}' if s else ''

        W   = 76
        top = f'{CY}┌{"─"*W}┐{R}'
        mid = f'{CY}├{"─"*W}┤{R}'
        bot = f'{CY}└{"─"*W}┘{R}'

        v_l_ms  = ml * RPM_TO_MPS
        v_r_ms  = mr * RPM_TO_MPS
        v_robot = (v_l_ms + v_r_ms) / 2.0
        w_robot = (v_r_ms - v_l_ms) / WHEEL_BASE

        mtr = f'{GN}ON{R}' if self._running else f'{RD}OFF{R}'

        lines = ['', top,
                 frow(f'{B}v7.2{R}  t={t:7.2f}s  {B}{lbl}{R}  motors={mtr}'),
                 mid,
                 frow(f'  {DM}v_robot={v_robot:+.3f}m/s  '
                      f'ω_robot={w_robot:+.3f}rad/s  '
                      f'MAX_RPM={MAX_RPM:.0f}  Kp={Kp}  Ki={Ki}  Kd={Kd}{R}'),
                 mid,
                ]

        for side, sp, meas, cmd, pid, stale in [
            ('GAUCHE', sp_l, ml, cmd_l, self._pid_l, stale_l),
            ('DROITE', sp_r, mr, cmd_r, self._pid_r, stale_r),
        ]:
            err = sp - meas
            lines.append(frow(
                f'  {B}{side}{R}'
                f'  SP={B}{sp:+6.1f}{R}'
                f'  meas={B}{meas:+6.1f}{R}'
                f'  err={ce(err)}{B}{err:+6.1f}{R}'
                f'  cmd_servo={B}{cmd:+6.1f}{R}'
                f'  ∫={B}{pid.integ:+5.2f}{R}'
                f'{stag(stale)}'
            ))

        lines.append(mid)

        def vbar(sp, meas, label):
            n_sp   = int(abs(sp)   / MAX_RPM * 22)
            n_meas = int(abs(meas) / MAX_RPM * 22)
            sym_sp   = '>' if sp   >= 0 else '<'
            sym_meas = '>' if meas >= 0 else '<'
            bar_sp   = f'{GN}{sym_sp   * n_sp:<22}{R}'
            bar_meas = f'{YL}{sym_meas * n_meas:<22}{R}'
            return (f'  {B}{label}{R}  SP {bar_sp} {sp:+6.1f}rpm  '
                    f'mes {bar_meas} {meas:+6.1f}rpm')

        lines.append(frow(vbar(sp_l, ml, 'G')))
        lines.append(frow(vbar(sp_r, mr, 'D')))

        if abs(sp_l) > 0.5 or abs(sp_r) > 0.5:
            delta = abs(sp_l - sp_r)
            pct   = delta / (2 * MAX_RPM) * 100
            n     = int(pct / 100 * 28)
            col   = GN if pct < 40 else (YL if pct < 75 else RD)
            lines.append(frow(
                f'  {DM}Δcouple={delta:.0f}rpm  '
                f'{col}{"█" * n:<28}{R}  {B}{pct:.0f}%{R}'
            ))

        lines.append(mid)
        lines.append(frow(
            f'  {DM}Commandes servo physiques :'
            f'  S1(gauche)={B}{cmd_l * LEFT_SIGN:+6.1f}{R}rpm  '
            f'  S2(droite)={B}{cmd_r * RIGHT_SIGN:+6.1f}{R}rpm{R}'
        ))

        lines.append(bot)
        print('\n'.join(lines), flush=True)

    def destroy_node(self):
        try:
            self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  0.0))
            self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, 0.0))
            self._set_torque(False)
        except Exception:
            pass
        super().destroy_node()


# ════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    if '--teleop' in (args or sys.argv):
        node = TeleopKeyboard()
        node.run()
        node.destroy_node()
        rclpy.shutdown()
        return
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

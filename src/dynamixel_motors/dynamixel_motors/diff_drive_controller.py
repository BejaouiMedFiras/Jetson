"""
diff_drive_controller.py  —  v8.0
4-MOTOR INDIVIDUAL PID CONTROL
═══════════════════════════════════════════════════════════════════════

v8.0 vs v7.2 :

  [4M-1] 4 PIDs individuels : FL (avant-gauche), RL (arrière-gauche),
          FR (avant-droit), RR (arrière-droit)

  [4M-2] Feedback encodeur individuel par moteur :
          present_velocity   → moteur A (avant)
          present_velocity_b → moteur B (arrière)

  [4M-3] MAX_RPM = 620 aligné avec ESP32 v5.0L / v5.1
          PROFILE_ACCEL = 800 identique à RAMP_RPM_S esp32

  [4M-4] Virage (j/l) : les 2 moteurs EXTÉRIEURS reçoivent 620rpm (plein)
          les 2 moteurs INTÉRIEURS reçoivent le profil sélectionné

  [4M-5] Diag affiche les 4 moteurs : SP / meas / err / cmd / intégrale
          + barre vitesse individuelle pour chaque moteur

  [4M-6] goal_velocity_b envoyé dans DynamixelCommand pour moteur B

  Convention physique (INCHANGÉE) :
    LEFT_SIGN  = +1   RIGHT_SIGN = -1
    Avancer = gauche +RPM, droite -RPM (montage miroir)

  Prérequis interface :
    DynamixelCommand  doit avoir : goal_velocity, goal_velocity_b
    DynamixelState    doit avoir : present_velocity, present_velocity_b
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
WHEEL_RADIUS  = 0.065          # m  (Ø130 mm)
WHEEL_BASE    = 0.20           # m  (entraxe gauche/droite)

# [4M-3] MAX_RPM = 620 — doit être IDENTIQUE à MAX_RPM_FREE dans l'ESP32
MAX_RPM       = 620.0

RPM_TO_MPS    = (2 * math.pi * WHEEL_RADIUS) / 60.0   # m/s par RPM

# Friction statique — seuil bas = 20/255*620 ≈ 48.6 rpm (MIN_PWM_DEADBAND ESP32)
FF_MIN_LINEAR   = 52.0         # rpm — au-dessus du deadband ESP32
FF_MIN_TURN_EXT = 52.0
FF_MIN_TURN_INT = 52.0

LEFT_SERVO_ID  = 1
RIGHT_SERVO_ID = 2

# ── Signes physiques — NE PAS MODIFIER ───────────────────────────────────
LEFT_SIGN  = +1.0
RIGHT_SIGN = -1.0

# ── PID (identique pour les 4 moteurs) ───────────────────────────────────
Kp = 1.8;  Ki = 2.5;  Kd = 0.025
PID_CLAMP   = 120.0
INTEG_CLAMP =  60.0

CONTROL_HZ  = 20.0
DIAG_HZ     =  1.0
TIMEOUT_S   =  5.0

# [4M-3] Doit être IDENTIQUE à RAMP_RPM_S dans l'ESP32
PROFILE_ACCEL_RPM_S2 = 800.0


# ════════════════════════════════════════════════════════════════════════════
# PROFILS DE VIRAGE  (RPM sur échelle MAX_RPM = 620)
# [4M-4] Roue extérieure = toujours 620rpm (PWM 255 = 12V = pleine vitesse)
# ════════════════════════════════════════════════════════════════════════════
TURN_PROFILES = {
    'a': (620.0,  360.0),   # virage doux
    'b': (620.0,  155.0),   # virage moyen
    'c': (620.0,   52.0),   # serré  ← DÉFAUT
    'd': (620.0, -310.0),   # très serré
    'e': (620.0, -620.0),   # rotation pure (pivot)
}
DEFAULT_PROFILE  = 'c'
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

def _turn_radius(ext, int_):
    if abs(ext - int_) < 0.1:
        return float('inf')
    return WHEEL_BASE / 2 * (ext + int_) / (ext - int_)


# ════════════════════════════════════════════════════════════════════════════
# PID
# ════════════════════════════════════════════════════════════════════════════
class WheelPID:
    def __init__(self, name: str):
        self.name  = name
        self.integ = 0.0
        self.deriv = 0.0
        self._prev = None
        self.last_err = 0.0

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
def _vcmd(sid: int, rpm_a: float, rpm_b: float) -> DynamixelCommand:
    """Commande vitesse pour moteur A ET moteur B d'un slave."""
    c = DynamixelCommand()
    c.servo_id          = sid
    c.operating_mode    = -1
    c.torque_enable     = -1
    c.goal_velocity     = float(rpm_a)      # moteur A (avant)
    c.goal_velocity_b   = float(rpm_b)      # moteur B (arrière)
    c.goal_position     = float('nan')
    c.goal_current      = float('nan')
    c.goal_pwm          = float('nan')
    c.profile_velocity      = float('nan')
    c.profile_acceleration  = float('nan')
    return c

def _icmd(sid: int) -> DynamixelCommand:
    """Commande d'initialisation."""
    c = DynamixelCommand()
    c.servo_id              = sid
    c.operating_mode        = 1
    c.torque_enable         = 0
    c.goal_velocity         = 0.0
    c.goal_velocity_b       = 0.0
    c.goal_position         = float('nan')
    c.goal_current          = float('nan')
    c.goal_pwm              = float('nan')
    c.profile_velocity      = float('nan')
    c.profile_acceleration  = float(PROFILE_ACCEL_RPM_S2)
    return c


# ════════════════════════════════════════════════════════════════════════════
# TÉLÉOP
# ════════════════════════════════════════════════════════════════════════════
_HELP = """
╔══════════════════════════════════════════════════════════════════════╗
║   TÉLÉOP v8.0  [MAX_RPM=620  4 moteurs individuels]                 ║
╠══════════════════════════════════════════════════════════════════════╣
║  i / ,   : avancer / reculer                                         ║
║  j       : ↰  VIRER GAUCHE  (droite=620rpm, gauche=profil)          ║
║  l       : ↱  VIRER DROITE  (gauche=620rpm, droite=profil)          ║
║  k       : ⛔  STOP                                                 ║
║  + / -   : vitesse linéaire                                          ║
║  a-e     : profil de virage                                          ║
╠══════════════════════════════════════════════════════════════════════╣
║  a  ext=620  int=+360  virage doux                                   ║
║  b  ext=620  int=+155  virage moyen                                  ║
║  c  ext=620  int= +52  serré  ← DÉFAUT                              ║
║  d  ext=620  int=-310  très serré                                    ║
║  e  ext=620  int=-620  pivot sur place                               ║
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
        self._lin  = 0.50          # m/s (≈ 73 rpm sur roue Ø130)
        self._prof = DEFAULT_PROFILE
        print(_HELP)
        self._status()

    def _status(self):
        ext, int_ = TURN_PROFILES[self._prof]
        r = _turn_radius(ext, int_)
        r_str = f'{r:.2f}m' if r != float('inf') and abs(r) < 9 else 'pivot'
        print(f'  Profil [{self._prof.upper()}]  ext={ext:+.0f}  int={int_:+.0f}  '
              f'R≈{r_str}  lin={self._lin:.2f}m/s')

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
                    print(f'  AVANCE  {self._lin:.2f}m/s → ≈{mps_to_rpm(self._lin):.0f}rpm')
                elif k == ',':
                    self._tx(-self._lin, 0.0)
                    print(f'  RECULE  {self._lin:.2f}m/s → ≈{mps_to_rpm(self._lin):.0f}rpm')
                elif k == 'j':
                    idx = list(TURN_PROFILES).index(self._prof) + 1
                    self._tx(0.0, float(idx))
                    ext, int_ = TURN_PROFILES[self._prof]
                    print(f'  GAUCHE [{self._prof.upper()}]  '
                          f'FL/RL(int)={int_:+.0f}rpm  FR/RR(ext)={ext:+.0f}rpm')
                elif k == 'l':
                    idx = list(TURN_PROFILES).index(self._prof) + 1
                    self._tx(0.0, -float(idx))
                    ext, int_ = TURN_PROFILES[self._prof]
                    print(f'  DROITE [{self._prof.upper()}]  '
                          f'FL/RL(ext)={ext:+.0f}rpm  FR/RR(int)={int_:+.0f}rpm')
                elif k.lower() in TURN_PROFILES:
                    self._prof = k.lower(); print(); self._status()
                elif k == '+':
                    self._lin = _clamp(self._lin + 0.05, 0.05, 1.50); self._status()
                elif k == '-':
                    self._lin = _clamp(self._lin - 0.05, 0.05, 1.50); self._status()
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
# CONTRÔLEUR PRINCIPAL  — 4 MOTEURS INDIVIDUELS
# ════════════════════════════════════════════════════════════════════════════
class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        # ── Setpoints (4 moteurs) ─────────────────────────────────────────
        self._sp_fl = 0.0    # avant-gauche  (slave1, moteur A)
        self._sp_rl = 0.0    # arrière-gauche(slave1, moteur B)
        self._sp_fr = 0.0    # avant-droit   (slave2, moteur A)
        self._sp_rr = 0.0    # arrière-droit (slave2, moteur B)

        # ── Mesures encodeurs (4 moteurs) ────────────────────────────────
        self._meas_fl = 0.0
        self._meas_rl = 0.0
        self._meas_fr = 0.0
        self._meas_rr = 0.0

        # ── Commandes calculées (pour affichage) ─────────────────────────
        self._cmd_fl = 0.0
        self._cmd_rl = 0.0
        self._cmd_fr = 0.0
        self._cmd_rr = 0.0

        # ── 4 PIDs individuels ────────────────────────────────────────────
        self._pid_fl = WheelPID('FL')
        self._pid_rl = WheelPID('RL')
        self._pid_fr = WheelPID('FR')
        self._pid_rr = WheelPID('RR')

        self._lbl         = 'ATTENTE'
        self._lock        = threading.Lock()
        self._initialized = False
        self._running     = False
        self._last_t      = self.get_clock().now()
        self._t0          = time.monotonic()

        # Timestamps feedback (pour détection stale)
        self._st_l = 0.0
        self._st_r = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Publishers / clients ─────────────────────────────────────────
        self.pub_l = self.create_publisher(
            DynamixelCommand, f'/servo{LEFT_SERVO_ID}/command',  10)
        self.pub_r = self.create_publisher(
            DynamixelCommand, f'/servo{RIGHT_SERVO_ID}/command', 10)
        self.tor_l = self.create_client(SetBool, f'/servo{LEFT_SERVO_ID}/set_torque')
        self.tor_r = self.create_client(SetBool, f'/servo{RIGHT_SERVO_ID}/set_torque')

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel',        self._on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._on_cmd, 10)
        self.create_subscription(
            DynamixelState, f'/servo{LEFT_SERVO_ID}/state',
            self._on_st_l, qos)
        self.create_subscription(
            DynamixelState, f'/servo{RIGHT_SERVO_ID}/state',
            self._on_st_r, qos)
        self.create_service(Empty, '/reset_pid', self._on_reset)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0,               self._init_motors)
        self.create_timer(0.1,               self._watchdog)
        self.create_timer(1.0 / CONTROL_HZ,  self._control_loop)
        self.create_timer(1.0 / DIAG_HZ,     self._print_diag)

        print('\n\033[1m\033[36m'
              '╔══════════════════════════════════════════════════════════════════╗\n'
              '║  diff_drive v8.0  —  4 moteurs individuels  MAX_RPM=620         ║\n'
             f'║  LEFT_SIGN={LEFT_SIGN:+.0f}  RIGHT_SIGN={RIGHT_SIGN:+.0f}  '
             f'FF_MIN={FF_MIN_LINEAR:.0f}rpm  Kp={Kp} Ki={Ki} Kd={Kd}         ║\n'
              '║  FL=avant-gauche  RL=arrière-gauche  FR=avant-droit  RR=arrière-droit ║\n'
              '╚══════════════════════════════════════════════════════════════════╝'
              '\033[0m', flush=True)

    # ────────────────────────────────────────────────────────────────────────
    # CALLBACKS ÉTAT
    # ────────────────────────────────────────────────────────────────────────
    def _on_reset(self, req, res):
        for pid in (self._pid_fl, self._pid_rl, self._pid_fr, self._pid_rr):
            pid.reset()
        self.get_logger().info('4 PIDs réinitialisés')
        return res

    def _on_st_l(self, msg: DynamixelState):
        """Feedback slave 1 : moteur A (avant-gauche) + moteur B (arrière-gauche)."""
        with self._lock:
            # present_velocity   = moteur A (avant)
            # present_velocity_b = moteur B (arrière)
            self._meas_fl = msg.present_velocity   * LEFT_SIGN
            self._meas_rl = msg.present_velocity_b * LEFT_SIGN
            self._st_l    = time.monotonic()

    def _on_st_r(self, msg: DynamixelState):
        """Feedback slave 2 : moteur A (avant-droit) + moteur B (arrière-droit)."""
        with self._lock:
            self._meas_fr = msg.present_velocity   * RIGHT_SIGN
            self._meas_rr = msg.present_velocity_b * RIGHT_SIGN
            self._st_r    = time.monotonic()

    # ────────────────────────────────────────────────────────────────────────
    # CALLBACK CMD_VEL
    # ────────────────────────────────────────────────────────────────────────
    def _on_cmd(self, msg: Twist):
        if not self._initialized:
            return
        self._last_t = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # ── Virage téléop (|w| encodé 1..5, v≈0) ────────────────────────
        # [4M-4] Les 2 roues EXTÉRIEURES = 620rpm  |  les 2 INTÉRIEURES = profil
        if 0.5 <= abs(w) <= 5.5 and abs(v) < 0.05:
            idx      = int(round(abs(w)))
            prof_key = _PROFILE_MAP.get(idx, DEFAULT_PROFILE)
            rpm_ext, rpm_int = TURN_PROFILES[prof_key]

            if w > 0:   # VIRER GAUCHE : roues droites extérieures
                sp_fl = +rpm_int   # gauche intérieure
                sp_rl = +rpm_int
                sp_fr = +rpm_ext   # droite extérieure
                sp_rr = +rpm_ext
                label = (f'GAUCHE [{prof_key.upper()}]  '
                         f'FL/RL={sp_fl:+.0f}rpm(int)  '
                         f'FR/RR={sp_fr:+.0f}rpm(ext)')
            else:        # VIRER DROITE : roues gauches extérieures
                sp_fl = +rpm_ext   # gauche extérieure
                sp_rl = +rpm_ext
                sp_fr = +rpm_int   # droite intérieure
                sp_rr = +rpm_int
                label = (f'DROITE [{prof_key.upper()}]  '
                         f'FL/RL={sp_fl:+.0f}rpm(ext)  '
                         f'FR/RR={sp_fr:+.0f}rpm(int)')

        # ── Ligne droite / commande ROS standard ─────────────────────────
        else:
            v_l  = v - w * WHEEL_BASE / 2.0
            v_r  = v + w * WHEEL_BASE / 2.0
            base = _clamp(mps_to_rpm(v_l), -MAX_RPM, MAX_RPM)
            sp_fl = sp_rl = base
            base  = _clamp(mps_to_rpm(v_r), -MAX_RPM, MAX_RPM)
            sp_fr = sp_rr = base
            label = (f'DROIT  v={v:+.3f}m/s  '
                     f'FL/RL={sp_fl:+.0f}  FR/RR={sp_fr:+.0f}rpm')

        with self._lock:
            self._sp_fl = sp_fl; self._sp_rl = sp_rl
            self._sp_fr = sp_fr; self._sp_rr = sp_rr
            self._lbl   = label

        if not self._running and any(abs(s) > 0.1
                for s in (sp_fl, sp_rl, sp_fr, sp_rr)):
            self._set_torque(True)
            self._running = True

    # ────────────────────────────────────────────────────────────────────────
    # BOUCLE DE CONTRÔLE — 4 PIDs
    # ────────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self._initialized or not self._running:
            return
        dt = 1.0 / CONTROL_HZ

        with self._lock:
            sp_fl = self._sp_fl; sp_rl = self._sp_rl
            sp_fr = self._sp_fr; sp_rr = self._sp_rr
            m_fl  = self._meas_fl; m_rl = self._meas_rl
            m_fr  = self._meas_fr; m_rr = self._meas_rr

        def resolve(sp, meas, pid, ff_min):
            if abs(sp) < 0.3:
                pid.reset()
                return 0.0
            ff = _apply_ff_min(sp, ff_min)
            return _clamp(ff + pid.update(sp, meas, dt), -MAX_RPM, MAX_RPM)

        # Détecter si virage (roues gauches et droites ont des SP différents)
        is_turn = abs(sp_fl - sp_fr) > 5.0

        if is_turn:
            left_is_ext = abs(sp_fl) >= abs(sp_fr)
            ff_l = FF_MIN_TURN_EXT if left_is_ext else FF_MIN_TURN_INT
            ff_r = FF_MIN_TURN_INT if left_is_ext else FF_MIN_TURN_EXT
        else:
            ff_l = ff_r = FF_MIN_LINEAR

        cmd_fl = resolve(sp_fl, m_fl, self._pid_fl, ff_l)
        cmd_rl = resolve(sp_rl, m_rl, self._pid_rl, ff_l)
        cmd_fr = resolve(sp_fr, m_fr, self._pid_fr, ff_r)
        cmd_rr = resolve(sp_rr, m_rr, self._pid_rr, ff_r)

        # Appliquer les signes physiques
        cmd_fl_s = cmd_fl * LEFT_SIGN
        cmd_rl_s = cmd_rl * LEFT_SIGN
        cmd_fr_s = cmd_fr * RIGHT_SIGN
        cmd_rr_s = cmd_rr * RIGHT_SIGN

        with self._lock:
            self._cmd_fl = cmd_fl
            self._cmd_rl = cmd_rl
            self._cmd_fr = cmd_fr
            self._cmd_rr = cmd_rr

        # Slave 1 : moteur A (avant-gauche) + moteur B (arrière-gauche)
        self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  cmd_fl_s, cmd_rl_s))
        # Slave 2 : moteur A (avant-droit)  + moteur B (arrière-droit)
        self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, cmd_fr_s, cmd_rr_s))

    # ────────────────────────────────────────────────────────────────────────
    # INITIALISATION MOTEURS
    # ────────────────────────────────────────────────────────────────────────
    def _init_motors(self):
        if self._initialized:
            return
        if not (self.tor_l.wait_for_service(0.05) and
                self.tor_r.wait_for_service(0.05)):
            return
        for sid, pub in [(LEFT_SERVO_ID,  self.pub_l),
                         (RIGHT_SERVO_ID, self.pub_r)]:
            pub.publish(_icmd(sid))
        time.sleep(0.4)
        self._set_torque(True)
        self._initialized = True
        self._running     = True
        self.get_logger().info('Moteurs initialisés v8.0 — 4 PIDs actifs')

    def _set_torque(self, en: bool):
        for c in (self.tor_l, self.tor_r):
            r = SetBool.Request(); r.data = en; c.call_async(r)
        if not en:
            for pid in (self._pid_fl, self._pid_rl,
                        self._pid_fr, self._pid_rr):
                pid.reset()

    # ────────────────────────────────────────────────────────────────────────
    # WATCHDOG
    # ────────────────────────────────────────────────────────────────────────
    def _watchdog(self):
        if not self._initialized:
            return
        elapsed = (self.get_clock().now() - self._last_t).nanoseconds / 1e9
        if elapsed > TIMEOUT_S and self._running:
            with self._lock:
                self._sp_fl = self._sp_rl = 0.0
                self._sp_fr = self._sp_rr = 0.0
                self._lbl   = 'WATCHDOG — STOP'
            self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  0.0, 0.0))
            self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, 0.0, 0.0))
            self._set_torque(False)
            self._running = False

    # ────────────────────────────────────────────────────────────────────────
    # AFFICHAGE DIAGNOSTIC — 4 MOTEURS
    # ────────────────────────────────────────────────────────────────────────
    def _print_diag(self):
        now = time.monotonic()
        t   = now - self._t0

        with self._lock:
            sp_fl = self._sp_fl; sp_rl = self._sp_rl
            sp_fr = self._sp_fr; sp_rr = self._sp_rr
            m_fl  = self._meas_fl; m_rl = self._meas_rl
            m_fr  = self._meas_fr; m_rr = self._meas_rr
            c_fl  = self._cmd_fl;  c_rl = self._cmd_rl
            c_fr  = self._cmd_fr;  c_rr = self._cmd_rr
            lbl   = self._lbl

        stale_l = (now - self._st_l > 0.5) if self._st_l else True
        stale_r = (now - self._st_r > 0.5) if self._st_r else True

        # Couleurs ANSI
        R  = '\033[0m'
        B  = '\033[1m'
        CY = '\033[36m'
        GN = '\033[32m'
        YL = '\033[33m'
        RD = '\033[31m'
        DM = '\033[2m'
        MG = '\033[35m'

        W = 82

        def strip_ansi(s):
            import re
            return re.sub(r'\033\[[0-9;]*m', '', s)

        def pad(s, w):
            return s + ' ' * max(0, w - len(strip_ansi(s)))

        def frow(c):
            return f'{CY}│{R} {pad(c, W-1)} {CY}│{R}'

        def ce(e):
            return GN if abs(e) < 15 else (YL if abs(e) < 50 else RD)

        def stag(s):
            return f' {RD}[NO FEEDBACK]{R}' if s else ''

        def motor_row(name, sp, meas, cmd, pid, stale, color):
            err = sp - meas
            return frow(
                f'  {color}{B}{name:2s}{R}'
                f'  SP={B}{sp:+6.1f}{R}'
                f'  meas={B}{meas:+6.1f}{R}'
                f'  err={ce(err)}{B}{err:+6.1f}{R}'
                f'  cmd={B}{cmd:+6.1f}{R}'
                f'  ∫={B}{pid.integ:+5.2f}{R}'
                f'{stag(stale)}'
            )

        def vbar(name, sp, meas, color):
            n_sp   = int(min(abs(sp),   MAX_RPM) / MAX_RPM * 18)
            n_meas = int(min(abs(meas), MAX_RPM) / MAX_RPM * 18)
            sym_sp   = '▶' if sp   >= 0 else '◀'
            sym_meas = '▶' if meas >= 0 else '◀'
            bar_sp   = f'{GN}{sym_sp   * n_sp:<18}{R}'
            bar_meas = f'{YL}{sym_meas * n_meas:<18}{R}'
            return frow(
                f'  {color}{B}{name:2s}{R}'
                f'  SP {bar_sp} {sp:+6.1f}'
                f'  mes {bar_meas} {meas:+6.1f}rpm'
            )

        mtr = f'{GN}ON{R}' if self._running else f'{RD}OFF{R}'
        top = f'{CY}┌{"─"*W}┐{R}'
        mid = f'{CY}├{"─"*W}┤{R}'
        bot = f'{CY}└{"─"*W}┘{R}'

        v_l   = ((m_fl + m_rl) / 2.0) * RPM_TO_MPS
        v_r   = ((m_fr + m_rr) / 2.0) * RPM_TO_MPS
        v_rob = (v_l + v_r) / 2.0
        w_rob = (v_r - v_l) / WHEEL_BASE

        lines = ['', top,
                 frow(f'{B}v8.0{R}  t={t:7.2f}s  {B}{lbl}{R}  motors={mtr}'),
                 mid,
                 frow(f'  {DM}v_robot={v_rob:+.3f}m/s  ω={w_rob:+.3f}rad/s  '
                      f'MAX_RPM={MAX_RPM:.0f}  Kp={Kp}  Ki={Ki}  Kd={Kd}{R}'),
                 mid,
                 frow(f'  {B}{CY}── CÔTÉ GAUCHE (slave 1) ──{R}'
                      f'  feedback: {GN if not stale_l else RD}'
                      f'{"OK" if not stale_l else "STALE"}{R}'),
                 motor_row('FL', sp_fl, m_fl, c_fl, self._pid_fl, stale_l, GN),
                 motor_row('RL', sp_rl, m_rl, c_rl, self._pid_rl, stale_l, GN),
                 mid,
                 frow(f'  {B}{MG}── CÔTÉ DROIT  (slave 2) ──{R}'
                      f'  feedback: {GN if not stale_r else RD}'
                      f'{"OK" if not stale_r else "STALE"}{R}'),
                 motor_row('FR', sp_fr, m_fr, c_fr, self._pid_fr, stale_r, MG),
                 motor_row('RR', sp_rr, m_rr, c_rr, self._pid_rr, stale_r, MG),
                 mid,
                 vbar('FL', sp_fl, m_fl, GN),
                 vbar('RL', sp_rl, m_rl, GN),
                 vbar('FR', sp_fr, m_fr, MG),
                 vbar('RR', sp_rr, m_rr, MG),
                 mid,
                 frow(
                     f'  {DM}Commandes physiques :'
                     f'  S1→A={B}{c_fl * LEFT_SIGN:+.1f}{R}'
                     f'  S1→B={B}{c_rl * LEFT_SIGN:+.1f}{R}'
                     f'  S2→A={B}{c_fr * RIGHT_SIGN:+.1f}{R}'
                     f'  S2→B={B}{c_rr * RIGHT_SIGN:+.1f}{R}rpm{R}'
                 ),
                 bot,
                ]

        print('\n'.join(lines), flush=True)

    # ────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self.pub_l.publish(_vcmd(LEFT_SERVO_ID,  0.0, 0.0))
            self.pub_r.publish(_vcmd(RIGHT_SERVO_ID, 0.0, 0.0))
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

# DynamixelState.msg
# État retourné par un servo Dynamixel (Protocol 2.0)
# Publié en continu par le nœud ROS2 (bulk read périodique)

# ── Identification ───────────────────────────────────────────────────
uint8   servo_id

# ── Mode courant ─────────────────────────────────────────────────────
uint8   operating_mode   # 1=VEL 3=POS 4=EXTPOS 5=CURRPOS 16=PWM

# ── État couple ──────────────────────────────────────────────────────
bool    torque_enabled

# ── Mesures moteur A (avant) ─────────────────────────────────────────
float32 present_position      # [degrés]
float32 present_velocity      # [tr/min]  moteur A — CT_NOW_VEL  (128)
float32 present_current       # [mA]
float32 present_pwm           # [%]
float32 present_voltage       # [V]
uint8   present_temperature   # [°C]

# ── Mesures moteur B (arrière) ───────────────────────────────────────
float32 present_velocity_b    # [tr/min]  moteur B — CT_NOW_VEL_B (132)
float32 present_current_b     # [mA]      moteur B (futur)

# ── Valeurs de consigne courantes ────────────────────────────────────
float32 goal_position
float32 goal_velocity         # [tr/min]  moteur A
float32 goal_velocity_b       # [tr/min]  moteur B
float32 goal_current
float32 goal_pwm

# ── Alarmes / erreurs ────────────────────────────────────────────────
uint8   hardware_error        # 0 = OK
bool    is_alive              # false si timeout bulk read

# ── Méta ─────────────────────────────────────────────────────────────
builtin_interfaces/Time stamp
float32 push_rate_hz          # fréquence réelle des lectures bulk

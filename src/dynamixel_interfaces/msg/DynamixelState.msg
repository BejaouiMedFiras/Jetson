# DynamixelState.msg
# État retourné par un servo Dynamixel (Protocol 2.0)
# Publié en continu par le nœud ROS2 (bulk read périodique)

# ── Identification ───────────────────────────────────────────────────
uint8   servo_id

# ── Mode courant ─────────────────────────────────────────────────────
uint8   operating_mode   # 1=VEL 3=POS 4=EXTPOS 5=CURRPOS 16=PWM

# ── État couple ──────────────────────────────────────────────────────
bool    torque_enabled

# ── Mesures en temps réel ────────────────────────────────────────────
float32 present_position      # [degrés]
float32 present_velocity      # [tr/min]
float32 present_current       # [mA]
float32 present_pwm           # [%]
float32 present_voltage       # [V]
uint8   present_temperature   # [°C]

# ── Valeurs de consigne courantes ────────────────────────────────────
float32 goal_position
float32 goal_velocity
float32 goal_current
float32 goal_pwm

# ── Alarmes / erreurs ────────────────────────────────────────────────
# Hardware Error Status bits (registre 0x46):
#   bit 0: Input voltage error
#   bit 2: Overheating error
#   bit 3: Motor encoder error
#   bit 4: Electrical shock error
#   bit 5: Overload error
uint8   hardware_error        # 0 = OK
bool    is_alive              # false si timeout bulk read

# ── Méta ─────────────────────────────────────────────────────────────
builtin_interfaces/Time stamp
float32 push_rate_hz          # fréquence réelle des lectures bulk

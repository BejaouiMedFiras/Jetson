# DynamixelCommand.msg
# Commande envoyée à un ou plusieurs servos Dynamixel (Protocol 2.0)

# ── Cible ────────────────────────────────────────────────────────────
uint8   servo_id          # 0 = broadcast, 1-252 = unicast

# ── Mode ─────────────────────────────────────────────────────────────
int8    operating_mode    # -1 = ne pas changer | 1=VEL 3=POS 16=PWM

# ── Couple ───────────────────────────────────────────────────────────
int8    torque_enable     # -1 = ne pas changer | 0 = OFF | 1 = ON

# ── Commandes moteur A (avant) ───────────────────────────────────────
float32 goal_position     # [degrés] mode 3/4/5  (NaN = ignorer)
float32 goal_velocity     # [tr/min] moteur A — CT_GOAL_VEL  (104)  (NaN = ignorer)
float32 goal_current      # [mA]     mode 5       (NaN = ignorer)
float32 goal_pwm          # [%]      mode 16      (NaN = ignorer)

# ── Commande moteur B (arrière) ──────────────────────────────────────
float32 goal_velocity_b   # [tr/min] moteur B — CT_GOAL_VEL_B (108)  (NaN = ignorer)
                          # Si NaN → moteur B reçoit la même valeur que moteur A

# ── Profil de mouvement ──────────────────────────────────────────────
float32 profile_velocity      # [tr/min]  0 = désactivé  (NaN = ignorer)
float32 profile_acceleration  # [tr/min²] 0 = désactivé  (NaN = ignorer)

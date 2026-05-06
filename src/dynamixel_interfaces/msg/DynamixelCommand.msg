# DynamixelCommand.msg
# Commande envoyée à un ou plusieurs servos Dynamixel (Protocol 2.0)
#
# Modes de contrôle (operating_mode):
#   1  = VELOCITY       — contrôle vitesse pure
#   3  = POSITION       — contrôle position (0-4095 ou 0-360°)
#   4  = EXT_POSITION   — position étendue multi-tours
#   5  = CURR_POSITION  — position avec limite de courant
#  16  = PWM            — contrôle PWM direct

# ── Cible ────────────────────────────────────────────────────────────
uint8   servo_id          # 0 = broadcast (sync write tous), 1-252 = unicast

# ── Mode ─────────────────────────────────────────────────────────────
int8    operating_mode    # -1 = ne pas changer, sinon voir codes ci-dessus

# ── Couple ───────────────────────────────────────────────────────────
int8    torque_enable     # -1 = ne pas changer | 0 = OFF | 1 = ON

# ── Commandes de mouvement ───────────────────────────────────────────
# Utilisez uniquement le champ correspondant à votre mode courant.
# Les autres seront ignorés.

float32 goal_position     # [degrés] mode 3/4/5  (NaN = ignorer)
float32 goal_velocity     # [tr/min] mode 1       (NaN = ignorer)  
                          #          mode 3/4/5 = vitesse de profil
float32 goal_current      # [mA]     mode 5       (NaN = ignorer)
float32 goal_pwm          # [%]      mode 16      (NaN = ignorer)

# ── Profil de mouvement ──────────────────────────────────────────────
float32 profile_velocity      # [tr/min]  0 = désactivé  (NaN = ignorer)
float32 profile_acceleration  # [tr/min²] 0 = désactivé  (NaN = ignorer)

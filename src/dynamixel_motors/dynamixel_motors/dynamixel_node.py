#!/usr/bin/env python3
# ============================================
# DYNAMIXEL ROS2 NODE v1.1
#
# CORRECTIONS v1.1 (fixes vs v1.0):
#   [FIX G] _cmd_callback: ordre des opérations corrigé
#       AVANT: mode → torque → profil → goal
#              → torque activé AVANT que le mode soit propre
#              → ESP32 ignore le changement de mode si torque=ON
#              → rampSP jamais réinitialisé → moteur explose
#       APRÈS: désactiver torque si mode change → set_mode
#              (set_mode gère torque OFF/ON en interne)
#              → profil → torque explicite → goal
#
#   [FIX H] _cmd_callback: skip set_profile si aucune valeur utile
#       profile_acceleration=0.0 avec profile_velocity=nan ne doit
#       pas déclencher un write inutile sur le bus
#       → Guard: has_profile = True seulement si au moins une
#         valeur est > 0 et non-nan
#
#   [FIX I] _cmd_callback: goal_velocity transmis en RPM
#       msg.goal_velocity est déjà en RPM → servo.velocity(rpm)
#       → pas de double conversion (inchangé vs v1.0, confirmé OK)
#
# Topics publiés:
#   /servo{id}/state  [DynamixelState]  — état temps réel
#
# Topics souscrits:
#   /servo{id}/command    [DynamixelCommand]  — commande unicast
#   /broadcast/command    [DynamixelCommand]  — commande tous servos
#
# Services:
#   /servo{id}/set_torque    — active/désactive le couple
#   /servo{id}/emergency_stop
#   /emergency_stop          — arrêt d'urgence global
#
# Paramètres ROS2:
#   port        (string)    — /dev/ttyUSB0
#   baudrate    (int)       — 115200
#   servo_ids   (int[])     — [1, 2]
#   read_hz     (double)    — 10.0
#   keepalive   (double)    — 1.0
# ============================================

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool, Trigger
from builtin_interfaces.msg import Time as RosTime

from dynamixel_interfaces.msg import DynamixelCommand, DynamixelState
from .dynamixel_driver import (
    DynamixelBus, Servo, KeepAlive, Mode, Conv, CT
)


def _ros_time(node: Node) -> RosTime:
    t = node.get_clock().now()
    msg = RosTime()
    msg.sec, msg.nanosec = divmod(t.nanoseconds, 10**9)
    return msg


# ─────────────────────────────────────────────────────────────────────
# NODE PRINCIPAL
# ─────────────────────────────────────────────────────────────────────
class DynamixelNode(Node):

    def __init__(self):
        super().__init__('dynamixel_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate',   115200)
        self.declare_parameter('servo_ids', [1, 2])
        self.declare_parameter('read_hz',    10.0)
        self.declare_parameter('keepalive',  1.0)

        port      = self.get_parameter('port').value
        baudrate  = self.get_parameter('baudrate').value
        servo_ids = list(self.get_parameter('servo_ids').value)
        read_hz   = self.get_parameter('read_hz').value
        ka_hz     = self.get_parameter('keepalive').value

        self.get_logger().info(
            f"[DXL] Init bus {port} @ {baudrate} baud | "
            f"servos={servo_ids} | read={read_hz}Hz"
        )

        self.bus = DynamixelBus(
            port=port, baudrate=baudrate,
            servo_ids=servo_ids, read_hz=read_hz
        )
        self.servo_objs: dict[int, Servo] = {
            sid: Servo(self.bus, sid) for sid in servo_ids
        }

        self.ka = KeepAlive(
            self.bus, list(self.servo_objs.values()), interval=ka_hz)
        self.ka.start()

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_state: dict[int, rclpy.publisher.Publisher] = {}
        for sid in servo_ids:
            self._pub_state[sid] = self.create_publisher(
                DynamixelState, f'/servo{sid}/state', qos_state)

        for sid in servo_ids:
            self.create_subscription(
                DynamixelCommand, f'/servo{sid}/command',
                lambda msg, s=sid: self._cmd_callback(msg, s), 10)

        self.create_subscription(
            DynamixelCommand, '/broadcast/command',
            self._broadcast_callback, 10)

        for sid in servo_ids:
            self.create_service(
                SetBool, f'/servo{sid}/set_torque',
                lambda req, res, s=sid: self._srv_torque(req, res, s))
            self.create_service(
                Trigger, f'/servo{sid}/emergency_stop',
                lambda req, res, s=sid: self._srv_estop_single(req, res, s))

        self.create_service(
            Trigger, '/emergency_stop', self._srv_estop_all)

        self._pub_timer = self.create_timer(
            1.0 / read_hz, self._publish_states)

        self.get_logger().info("[DXL] Nœud prêt")

    # ─────────────────────────────────────────────────────────────────
    # CALLBACK COMMANDE
    # ─────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: DynamixelCommand, sid: int):
        """
        [FIX G] Ordre correct des opérations:
          1. Si le mode change → désactiver le couple en premier
             (l'ESP32 IGNORE les changements de mode si torque=ON)
          2. set_mode() — gère torque OFF/ON en interne si nécessaire
          3. Profil — après que le mode soit établi, seulement si
             au moins une valeur est significative (>0, non-nan) [FIX H]
          4. Activer/désactiver le couple
          5. Envoyer la consigne de mouvement

        Cette séquence garantit que rampSP sur l'ESP32 est toujours
        réinitialisé dans la bonne unité (RPM pour VELOCITY, counts
        pour POSITION) avant que le couple soit activé.
        """
        servo = self.servo_objs.get(sid)
        if servo is None:
            self.get_logger().warn(f"[DXL] Servo {sid} inconnu")
            return

        # ── 1. Changement de mode ─────────────────────────────────
        # L'ESP32 exige que le couple soit OFF pour changer de mode.
        # set_mode() désactive le couple, change le mode, puis
        # réactive si nécessaire. On l'appelle seulement si le mode
        # change réellement pour éviter des transitions inutiles.
        mode_requested = msg.operating_mode
        if mode_requested >= 0:
            if mode_requested != servo.state.operating_mode:
                self.get_logger().info(
                    f"[DXL] S{sid} changement de mode "
                    f"{servo.state.operating_mode} → {mode_requested}"
                )
                # [FIX G] Forcer torque OFF avant le changement de mode,
                # même si msg.torque_enable ne le demande pas explicitement.
                # Cela garantit que l'ESP32 réinitialise rampSP correctement.
                if servo.state.torque_enabled:
                    servo.disable_torque()
                    time.sleep(0.05)   # laisse l'ESP32 traiter le torque-off
                servo.set_mode(mode_requested)
                # set_mode() attend 150ms × 2 en interne → mode stable ici
            else:
                self.get_logger().debug(
                    f"[DXL] S{sid} mode déjà {mode_requested}, pas de changement")

        # ── 2. Profil de mouvement ────────────────────────────────
        # [FIX H] N'écrire le profil que si au moins une valeur est
        # utile (>0 et non-nan). profile_acceleration=0.0 seul ne
        # doit pas déclencher un write: 0 sur CT_PROF_ACC est
        # valide (= utiliser défaut ESP32) mais inutile d'écrire
        # si l'utilisateur n'a pas fourni de vraie valeur.
        pv = msg.profile_velocity
        pa = msg.profile_acceleration

        pv_valid = (not math.isnan(pv)) and (pv > 0)
        pa_valid = (not math.isnan(pa)) and (pa > 0)

        if pv_valid or pa_valid:
            v = pv if pv_valid else servo.state.profile_velocity
            a = pa if pa_valid else servo.state.profile_acceleration
            self.get_logger().debug(
                f"[DXL] S{sid} set_profile vel={v:.1f}rpm/s accel={a:.1f}rpm/s²")
            servo.set_profile(v=v, a=a)

        # ── 3. Couple ─────────────────────────────────────────────
        # Appliquer APRÈS le mode et le profil pour que l'ESP32
        # initialise rampSP dans la bonne unité.
        if msg.torque_enable == 1:
            servo.enable_torque()
        elif msg.torque_enable == 0:
            servo.disable_torque()

        # ── 4. Consigne de mouvement ──────────────────────────────
        # Utiliser le mode courant (après changement éventuel)
        mode = servo.state.operating_mode

        if mode == Mode.VELOCITY:
            # [FIX I] msg.goal_velocity est en RPM → servo.velocity() l'accepte
            if not math.isnan(msg.goal_velocity):
                self.get_logger().debug(
                    f"[DXL] S{sid} goal_velocity={msg.goal_velocity:.1f} rpm")
                servo.velocity(msg.goal_velocity)

        elif mode in (Mode.POSITION, Mode.EXT_POSITION):
            if not math.isnan(msg.goal_position):
                servo.move_to(msg.goal_position)

        elif mode == Mode.CURR_POSITION:
            if not math.isnan(msg.goal_position):
                ma = msg.goal_current if not math.isnan(msg.goal_current) else 500.0
                servo.move_to_with_current(msg.goal_position, max_ma=ma)

        elif mode == Mode.PWM:
            if not math.isnan(msg.goal_pwm):
                servo.pwm(msg.goal_pwm)

        else:
            self.get_logger().warn(f"[DXL] S{sid} mode inconnu: {mode}")

    def _broadcast_callback(self, msg: DynamixelCommand):
        """Commande broadcast: appliquée à tous les servos."""
        for sid in self.servo_objs:
            self._cmd_callback(msg, sid)

    # ─────────────────────────────────────────────────────────────────
    # SERVICES
    # ─────────────────────────────────────────────────────────────────
    def _srv_torque(self, req: SetBool.Request,
                    res: SetBool.Response, sid: int) -> SetBool.Response:
        servo = self.servo_objs.get(sid)
        if servo is None:
            res.success = False
            res.message = f"Servo {sid} inconnu"
            return res
        ok = servo.enable_torque() if req.data else servo.disable_torque()
        res.success = ok
        res.message = f"S{sid} torque={'ON' if req.data else 'OFF'} {'OK' if ok else 'ERR'}"
        self.get_logger().info(res.message)
        return res

    def _srv_estop_single(self, req: Trigger.Request,
                           res: Trigger.Response, sid: int) -> Trigger.Response:
        self.bus.emergency_stop([sid])
        res.success = True
        res.message = f"S{sid} arrêt d'urgence — couple coupé"
        self.get_logger().warn(res.message)
        return res

    def _srv_estop_all(self, req: Trigger.Request,
                        res: Trigger.Response) -> Trigger.Response:
        self.bus.emergency_stop()
        res.success = True
        res.message = "ARRÊT D'URGENCE — couple coupé sur tous les servos"
        self.get_logger().error(res.message)
        return res

    # ─────────────────────────────────────────────────────────────────
    # PUBLICATION ÉTATS
    # ─────────────────────────────────────────────────────────────────
    def _publish_states(self):
        now = _ros_time(self)
        for sid, servo in self.servo_objs.items():
            s   = servo.state
            msg = DynamixelState()

            msg.stamp               = now
            msg.servo_id            = sid
            msg.operating_mode      = s.operating_mode
            msg.torque_enabled      = s.torque_enabled
            msg.present_position    = float(s.present_position)
            msg.present_velocity    = float(s.present_velocity)
            msg.present_current     = float(s.present_current)
            msg.present_pwm         = float(s.present_pwm)
            msg.present_voltage     = float(s.present_voltage)
            msg.present_temperature = int(s.present_temperature)
            msg.goal_position       = float(s.goal_position)
            msg.goal_velocity       = float(s.goal_velocity)
            msg.goal_current        = float(s.goal_current)
            msg.goal_pwm            = float(s.goal_pwm)
            msg.hardware_error      = int(s.hardware_error)
            msg.is_alive            = s.is_alive()
            msg.push_rate_hz        = float(s.push_rate)

            self._pub_state[sid].publish(msg)

    # ─────────────────────────────────────────────────────────────────
    # NETTOYAGE
    # ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("[DXL] Fermeture du nœud…")
        self.ka.stop()
        self.bus.emergency_stop()
        self.bus.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
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

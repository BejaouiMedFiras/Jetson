#!/usr/bin/env python3
# ============================================
# DYNAMIXEL ROS2 NODE v2.0
# 4 MOTEURS INDIVIDUELS — 2 par slave ESP32
#
# v2.0 vs v1.1 :
#
#   [4M-A] goal_velocity_b : moteur B (arrière) par slave
#       Lit msg.goal_velocity_b (DynamixelCommand) et écrit
#       CT_GOAL_VEL_B (registre 108) via servo.velocity_b().
#       Si goal_velocity_b est NaN → moteur B = moteur A (miroir).
#
#   [4M-B] present_velocity_b : feedback encodeur moteur B
#       Lit CT_NOW_VEL_B (registre 132) depuis servo.state.
#       Publié dans DynamixelState.present_velocity_b.
#
#   [4M-C] present_current_b : futur (si ACS712 moteur B)
#       Publié dans DynamixelState.present_current_b.
#
#   [4M-D] goal_velocity_b dans DynamixelState
#       Reflète la consigne en cours pour moteur B.
#
# Architecture matérielle :
#   Slave 1 (ID=1, GAUCHE) :
#     Moteur A = avant-gauche  (enc GPIO32/33, BTS7960 GPIO25/26)
#     Moteur B = arrière-gauche(enc GPIO34/39, BTS7960 GPIO18/19)
#   Slave 2 (ID=2, DROIT) :
#     Moteur A = avant-droit   (enc GPIO32/33, BTS7960 GPIO25/26)
#     Moteur B = arrière-droit (enc GPIO34/39, BTS7960 GPIO18/19)
#
# Registres ESP32 utilisés :
#   CT_GOAL_VEL   = 104  (moteur A, int32, unité = 0.229 rpm)
#   CT_GOAL_VEL_B = 108  (moteur B, int32, unité = 0.229 rpm)
#   CT_NOW_VEL    = 128  (moteur A, int32, unité = 0.229 rpm)
#   CT_NOW_VEL_B  = 132  (moteur B, int32, unité = 0.229 rpm)
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

        self.declare_parameter('port',      '/dev/ttyTHS1')
        self.declare_parameter('baudrate',  115200)
        self.declare_parameter('servo_ids', [1, 2])
        self.declare_parameter('read_hz',   20.0)
        self.declare_parameter('keepalive', 1.0)

        port      = self.get_parameter('port').value
        baudrate  = self.get_parameter('baudrate').value
        servo_ids = list(self.get_parameter('servo_ids').value)
        read_hz   = self.get_parameter('read_hz').value
        ka_hz     = self.get_parameter('keepalive').value

        self.get_logger().info(
            f"[DXL v2.0] bus {port} @ {baudrate} baud | "
            f"servos={servo_ids} | read={read_hz}Hz | 4 moteurs"
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

        self.get_logger().info(
            "[DXL v2.0] Prêt — FL/RL (slave1) + FR/RR (slave2)")

    # ─────────────────────────────────────────────────────────────────
    # CALLBACK COMMANDE
    # ─────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: DynamixelCommand, sid: int):
        """
        Traite une commande pour un slave :
          - goal_velocity   → moteur A (avant)   CT_GOAL_VEL  (104)
          - goal_velocity_b → moteur B (arrière) CT_GOAL_VEL_B(108)
            Si goal_velocity_b est NaN → moteur B = moteur A (miroir)
        """
        servo = self.servo_objs.get(sid)
        if servo is None:
            self.get_logger().warn(f"[DXL] Servo {sid} inconnu")
            return

        # ── 1. Changement de mode ─────────────────────────────────
        mode_requested = msg.operating_mode
        if mode_requested >= 0:
            if mode_requested != servo.state.operating_mode:
                self.get_logger().info(
                    f"[DXL] S{sid} mode "
                    f"{servo.state.operating_mode} → {mode_requested}"
                )
                if servo.state.torque_enabled:
                    servo.disable_torque()
                    time.sleep(0.05)
                servo.set_mode(mode_requested)

        # ── 2. Profil de mouvement ────────────────────────────────
        pv = msg.profile_velocity
        pa = msg.profile_acceleration
        pv_valid = (not math.isnan(pv)) and (pv > 0)
        pa_valid = (not math.isnan(pa)) and (pa > 0)
        if pv_valid or pa_valid:
            v = pv if pv_valid else servo.state.profile_velocity
            a = pa if pa_valid else servo.state.profile_acceleration
            servo.set_profile(v=v, a=a)

        # ── 3. Couple ─────────────────────────────────────────────
        if msg.torque_enable == 1:
            servo.enable_torque()
        elif msg.torque_enable == 0:
            servo.disable_torque()

        # ── 4. Consigne de mouvement ──────────────────────────────
        mode = servo.state.operating_mode

        if mode == Mode.VELOCITY:
            # [4M-A] Moteur A
            rpm_a = msg.goal_velocity
            if not math.isnan(rpm_a):
                servo.velocity(rpm_a)
                self.get_logger().debug(
                    f"[DXL] S{sid} motorA={rpm_a:+.1f}rpm")

            # [4M-A] Moteur B — si NaN → miroir moteur A
            rpm_b = msg.goal_velocity_b
            if math.isnan(rpm_b):
                rpm_b = rpm_a   # comportement par défaut : miroir A→B
            if not math.isnan(rpm_b):
                servo.velocity_b(rpm_b)
                self.get_logger().debug(
                    f"[DXL] S{sid} motorB={rpm_b:+.1f}rpm")

        elif mode in (Mode.POSITION, Mode.EXT_POSITION):
            if not math.isnan(msg.goal_position):
                servo.move_to(msg.goal_position)

        elif mode == Mode.CURR_POSITION:
            if not math.isnan(msg.goal_position):
                ma = msg.goal_current if not math.isnan(
                    msg.goal_current) else 500.0
                servo.move_to_with_current(msg.goal_position, max_ma=ma)

        elif mode == Mode.PWM:
            if not math.isnan(msg.goal_pwm):
                servo.pwm(msg.goal_pwm)

        else:
            self.get_logger().warn(f"[DXL] S{sid} mode inconnu: {mode}")

    def _broadcast_callback(self, msg: DynamixelCommand):
        for sid in self.servo_objs:
            self._cmd_callback(msg, sid)

    # ─────────────────────────────────────────────────────────────────
    # SERVICES
    # ─────────────────────────────────────────────────────────────────
    def _srv_torque(self, req, res, sid):
        servo = self.servo_objs.get(sid)
        if servo is None:
            res.success = False
            res.message = f"Servo {sid} inconnu"
            return res
        ok = servo.enable_torque() if req.data else servo.disable_torque()
        res.success = ok
        res.message = (f"S{sid} torque="
                       f"{'ON' if req.data else 'OFF'} "
                       f"{'OK' if ok else 'ERR'}")
        self.get_logger().info(res.message)
        return res

    def _srv_estop_single(self, req, res, sid):
        self.bus.emergency_stop([sid])
        res.success = True
        res.message = f"S{sid} arrêt d'urgence"
        self.get_logger().warn(res.message)
        return res

    def _srv_estop_all(self, req, res):
        self.bus.emergency_stop()
        res.success = True
        res.message = "ARRÊT D'URGENCE — tous servos"
        self.get_logger().error(res.message)
        return res

    # ─────────────────────────────────────────────────────────────────
    # PUBLICATION ÉTATS — 4 moteurs
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

            # ── Moteur A (avant) ──────────────────────────────────
            msg.present_position    = float(s.present_position)
            msg.present_velocity    = float(s.present_velocity)    # CT_NOW_VEL (128)
            msg.present_current     = float(s.present_current)
            msg.present_pwm         = float(s.present_pwm)
            msg.present_voltage     = float(s.present_voltage)
            msg.present_temperature = int(s.present_temperature)

            # ── Moteur B (arrière) — [4M-B] ──────────────────────
            msg.present_velocity_b  = float(s.present_velocity_b)  # CT_NOW_VEL_B (132)
            msg.present_current_b   = float(getattr(s, 'present_current_b', 0.0))

            # ── Consignes ─────────────────────────────────────────
            msg.goal_position       = float(s.goal_position)
            msg.goal_velocity       = float(s.goal_velocity)
            msg.goal_velocity_b     = float(getattr(s, 'goal_velocity_b', 0.0))
            msg.goal_current        = float(s.goal_current)
            msg.goal_pwm            = float(s.goal_pwm)

            # ── Santé ─────────────────────────────────────────────
            msg.hardware_error      = int(s.hardware_error)
            msg.is_alive            = s.is_alive()
            msg.push_rate_hz        = float(s.push_rate)

            self._pub_state[sid].publish(msg)

    # ─────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("[DXL] Fermeture…")
        self.ka.stop()
        self.bus.emergency_stop()
        self.bus.close()
        super().destroy_node()


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

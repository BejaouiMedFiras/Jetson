#!/usr/bin/env python3
# ============================================
# DYNAMIXEL DRIVER v3.0 — ESP32 daisy-chain / Python 3.6+
#
# v3.0 vs v2.5 — CORRECTIONS CRITIQUES :
#
#   [FIX CT-1] Table CT corrigée pour correspondre au firmware ESP32
#       AVANT (v2.5 — FAUX) :
#         PROFILE_ACCELERATION = 108   ← ESP32 a CT_GOAL_VEL_B ici !
#         PROFILE_VELOCITY     = 112
#         PRESENT_POSITION     = 132   ← ESP32 a CT_NOW_VEL_B ici !
#
#       APRÈS (v3.0 — CORRECT, correspondance firmware ESP32 v5.x) :
#         GOAL_VELOCITY_B      = 108   moteur B (arrière)
#         PROFILE_VELOCITY     = 112
#         PRESENT_VELOCITY_B   = 132   moteur B (arrière)
#
#       CONSÉQUENCES du bug v2.5 :
#         • set_profile(accel=800) → écrivait 800 rpm dans moteur B !
#         • PRESENT_POSITION lu = vitesse moteur B en RPM (incompréhensible)
#         • Moteur B ne recevait jamais de vraie commande de vitesse
#
#   [FIX CT-2] PROFILE_ACCELERATION supprimé
#       L'ESP32 custom n'a PAS de registre profile_acceleration.
#       Il gère l'accélération en interne (RAMP_RPM_S = 800 dans le firmware).
#       set_profile() n'envoie plus que PROFILE_VELOCITY (registre 112).
#
#   [4M-1] goal_velocity_b() — nouveau
#       Écrit CT_GOAL_VEL_B (108) pour contrôler le moteur B (arrière).
#
#   [4M-2] present_velocity_b dans ServoState
#       update_from_raw() lit CT_NOW_VEL_B (132) → present_velocity_b.
#
#   [4M-3] velocity_b() dans Servo
#       Façade pour goal_velocity_b().
#
#   [4M-4] KeepAlive envoie aussi goal_velocity_b
#       Pour éviter le timeout ESP32 sur le moteur B.
#
#   Conservés de v2.5 :
#   [FIX CRIT-1] Lectures séquentielles (pas GroupBulkRead)
#   [FIX CRIT-2] commanded_mode tracké séparément
#   [FIX CRIT-3] Bus mutex étendu à toute la séquence
#   [FIX CRIT-4] Timeout 2.0s
#
# Registres ESP32 v5.x (référence firmware) :
#   100: GOAL_PWM          int16
#   102: GOAL_CURRENT      int16
#   104: GOAL_VELOCITY     int32  moteur A (avant)
#   108: GOAL_VELOCITY_B   int32  moteur B (arrière)  ← était PROFILE_ACCEL
#   112: PROFILE_VELOCITY  int32
#   116: GOAL_POSITION     int32
#   124: PRESENT_PWM       int16
#   126: PRESENT_CURRENT   int16
#   128: PRESENT_VELOCITY  int32  moteur A (avant)
#   132: PRESENT_VELOCITY_B int32 moteur B (arrière)  ← était PRESENT_POSITION
#   144: PRESENT_VOLTAGE   int16
#   146: PRESENT_TEMPERATURE uint8
#   156: VEL_ZONE          uint8  (v1.9+)
#   157: STALL_FLAG        uint8  (v1.9+)
# ============================================

import time
import threading
import struct
import subprocess
from dataclasses import dataclass, field
from typing import List, Optional

try:
    from dynamixel_sdk import (
        PortHandler, PacketHandler,
        GroupSyncWrite,
        COMM_SUCCESS,
        DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
    )
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("[WARN] dynamixel_sdk non installé — simulation active")


# ════════════════════════════════════════════
# REGISTRES CONTROL TABLE — ESP32 v5.x
# [FIX CT-1] Corrigés pour correspondre au firmware ESP32 réel
# ════════════════════════════════════════════
class CT:
    OPERATING_MODE          = 11
    TORQUE_ENABLE           = 64
    HARDWARE_ERROR_STATUS   = 70
    GOAL_PWM                = 100   # int16
    GOAL_CURRENT            = 102   # int16
    GOAL_VELOCITY           = 104   # int32 — moteur A (avant)
    GOAL_VELOCITY_B         = 108   # int32 — moteur B (arrière)  [FIX CT-1]
    PROFILE_VELOCITY        = 112   # int32
    GOAL_POSITION           = 116   # int32
    PRESENT_PWM             = 124   # int16
    PRESENT_CURRENT         = 126   # int16
    PRESENT_VELOCITY        = 128   # int32 — moteur A (avant)
    PRESENT_VELOCITY_B      = 132   # int32 — moteur B (arrière)  [FIX CT-1]
    PRESENT_VOLTAGE         = 144   # int16
    PRESENT_TEMPERATURE     = 146   # uint8
    VEL_ZONE                = 156   # uint8 (v1.9+)
    STALL_FLAG              = 157   # uint8 (v1.9+)


# ════════════════════════════════════════════
# MODES DE FONCTIONNEMENT
# ════════════════════════════════════════════
class Mode:
    VELOCITY      = 1
    POSITION      = 3
    EXT_POSITION  = 4
    CURR_POSITION = 5
    PWM           = 16


# ════════════════════════════════════════════
# CONVERSIONS UNITÉS — ESP32 custom firmware
# ════════════════════════════════════════════
class Conv:
    POS_UNIT  = 360.0 / 4096.0   # deg/LSB
    VEL_UNIT  = 0.229             # rpm/LSB
    CUR_UNIT  = 2.69              # mA/LSB
    PWM_UNIT  = 0.113             # %/LSB
    VOLT_UNIT = 0.1               # V/LSB

    @staticmethod
    def deg_to_pos(d: float) -> int:
        return int(round(d / Conv.POS_UNIT))

    @staticmethod
    def pos_to_deg(p: int) -> float:
        if p > 0x7FFFFFFF: p -= 0x100000000
        return p * Conv.POS_UNIT

    @staticmethod
    def rpm_to_vel(r: float) -> int:
        return int(round(r / Conv.VEL_UNIT))

    @staticmethod
    def vel_to_rpm(v: int) -> float:
        if v > 0x7FFFFFFF: v -= 0x100000000
        return v * Conv.VEL_UNIT

    @staticmethod
    def ma_to_cur(m: float) -> int:
        return int(round(m / Conv.CUR_UNIT))

    @staticmethod
    def cur_to_ma(c: int) -> float:
        if c > 0x7FFF: c -= 0x10000
        return c * Conv.CUR_UNIT

    @staticmethod
    def pct_to_pwm(p: float) -> int:
        return int(round(p / Conv.PWM_UNIT))

    @staticmethod
    def pwm_to_pct(w: int) -> float:
        if w > 0x7FFF: w -= 0x10000
        return w * Conv.PWM_UNIT


# ════════════════════════════════════════════
# PLAGE DE LECTURE
# READ_START=64, READ_END=157 → 94 octets
# Couvre : TORQUE_ENABLE(64) → STALL_FLAG(157)
# Inclut : PRESENT_VELOCITY(128) ET PRESENT_VELOCITY_B(132)
# ════════════════════════════════════════════
READ_START = CT.TORQUE_ENABLE          # 64
READ_END   = CT.STALL_FLAG             # 157
READ_LEN   = READ_END - READ_START + 1 # 94 octets

HISTORY_SIZE = 50


def _u2s16(v: int) -> int:
    return v - 0x10000     if v > 0x7FFF      else v


def _u2s32(v: int) -> int:
    return v - 0x100000000 if v > 0x7FFFFFFF  else v


def _extract(raw: bytes, base: int, addr: int, ln: int) -> Optional[int]:
    off = addr - base
    if off < 0 or off + ln > len(raw):
        return None
    chunk = raw[off: off + ln]
    if ln == 1: return chunk[0]
    if ln == 2: return struct.unpack_from('<H', chunk)[0]
    if ln == 4: return struct.unpack_from('<I', chunk)[0]
    return None


# ════════════════════════════════════════════
# ÉTAT D'UN SERVO — 4 MOTEURS (A=avant, B=arrière)
# ════════════════════════════════════════════
@dataclass
class ServoState:
    servo_id: int

    # [FIX CRIT-2] Mode commandé vs mode lu
    commanded_mode:   int  = Mode.VELOCITY
    operating_mode:   int  = Mode.VELOCITY
    torque_enabled:   bool = False
    torque_commanded: bool = False

    # ── Moteur A (avant) ──────────────────────────────────────────────
    present_velocity:    float = 0.0   # rpm — CT_NOW_VEL  (128)
    present_current:     float = 0.0   # mA
    present_pwm:         float = 0.0   # %
    present_voltage:     float = 0.0   # V
    present_temperature: int   = 0
    present_position:    float = 0.0   # deg (si mode position)

    # ── Moteur B (arrière) ────────────────────────────────────────────
    present_velocity_b:  float = 0.0   # rpm — CT_NOW_VEL_B (132)  [4M-2]
    present_current_b:   float = 0.0   # mA  (futur ACS712 moteur B)

    # ── Consignes ─────────────────────────────────────────────────────
    goal_velocity:    float = 0.0   # rpm moteur A
    goal_velocity_b:  float = 0.0   # rpm moteur B  [4M-1]
    goal_position:    float = 0.0
    goal_current:     float = 0.0
    goal_pwm:         float = 0.0
    profile_velocity: float = 0.0   # [FIX CT-2] plus de profile_acceleration

    # ── Diagnostics ───────────────────────────────────────────────────
    hardware_error: int  = 0
    vel_zone:       int  = 1
    stall_flag:     bool = False

    # ── Statistiques ──────────────────────────────────────────────────
    last_seen:   float = 0.0
    push_count:  int   = 0
    push_rate:   float = 0.0
    read_errors: int   = 0

    hist_current:    List[float] = field(default_factory=lambda: [0.0] * HISTORY_SIZE)
    hist_velocity:   List[float] = field(default_factory=lambda: [0.0] * HISTORY_SIZE)
    hist_velocity_b: List[float] = field(default_factory=lambda: [0.0] * HISTORY_SIZE)

    _rate_t: float = field(default_factory=time.time, repr=False)
    _rate_n: int   = field(default=0, repr=False)

    TIMEOUT_S: float = 2.0   # [FIX CRIT-4]

    def update_from_raw(self, raw: bytes):
        def g(a, l): return _extract(raw, READ_START, a, l)

        te   = g(CT.TORQUE_ENABLE,         1)
        hw   = g(CT.HARDWARE_ERROR_STATUS,  1)
        pwm  = g(CT.PRESENT_PWM,           2)
        cur  = g(CT.PRESENT_CURRENT,       2)
        vel  = g(CT.PRESENT_VELOCITY,      4)   # moteur A (128)
        velb = g(CT.PRESENT_VELOCITY_B,    4)   # moteur B (132) [FIX CT-1]
        vlt  = g(CT.PRESENT_VOLTAGE,       2)
        tmp  = g(CT.PRESENT_TEMPERATURE,   1)
        vzn  = g(CT.VEL_ZONE,              1)
        stl  = g(CT.STALL_FLAG,            1)

        if te   is not None: self.torque_enabled     = bool(te)
        if hw   is not None: self.hardware_error     = hw
        if pwm  is not None: self.present_pwm        = Conv.pwm_to_pct(_u2s16(pwm))
        if cur  is not None: self.present_current    = Conv.cur_to_ma(_u2s16(cur))
        if vel  is not None: self.present_velocity   = Conv.vel_to_rpm(_u2s32(vel))
        if velb is not None: self.present_velocity_b = Conv.vel_to_rpm(_u2s32(velb))  # [4M-2]
        if vlt  is not None: self.present_voltage    = vlt * Conv.VOLT_UNIT
        if tmp  is not None: self.present_temperature = tmp
        if vzn  is not None: self.vel_zone           = vzn
        if stl  is not None: self.stall_flag         = bool(stl)

        # [FIX CRIT-2] Ne PAS mettre à jour operating_mode depuis la lecture
        # (CT_OPERATING_MODE=11 est hors de notre plage de lecture 64-157)

        self.last_seen   = time.time()
        self.push_count += 1
        self.read_errors = 0
        self._rate_n    += 1
        dt = time.time() - self._rate_t
        if dt >= 1.0:
            self.push_rate = self._rate_n / dt
            self._rate_n   = 0
            self._rate_t   = time.time()

        self.hist_current    = self.hist_current[1:]    + [self.present_current]
        self.hist_velocity   = self.hist_velocity[1:]   + [self.present_velocity]
        self.hist_velocity_b = self.hist_velocity_b[1:] + [self.present_velocity_b]

    def is_alive(self) -> bool:
        return self.last_seen > 0 and (time.time() - self.last_seen) < self.TIMEOUT_S


# ════════════════════════════════════════════
# BUS DYNAMIXEL
# ════════════════════════════════════════════
class DynamixelBus:
    PROTOCOL       = 2.0
    INTER_SERVO_MS = 0.020   # 20ms entre lectures séquentielles [FIX CRIT-1]

    def __init__(self, port: str = '/dev/ttyTHS1',
                 baudrate: int = 115200,
                 servo_ids: list = None,
                 read_hz: float = 25.0):
        self.port      = port
        self.baudrate  = baudrate
        self.servo_ids = servo_ids or [1, 2]
        self.read_hz   = read_hz
        self.lock      = threading.Lock()   # [FIX CRIT-3]
        self._running  = False
        self.servos    = {sid: ServoState(servo_id=sid) for sid in self.servo_ids}

        if not SDK_AVAILABLE:
            print("[DXL] Mode simulation (SDK absent)")
            self._sim = True
            self._running = True
            threading.Thread(target=self._sim_loop, daemon=True).start()
            return

        self._sim = False
        self.ph   = PortHandler(port)
        self.pkt  = PacketHandler(self.PROTOCOL)

        if not self.ph.openPort():
            raise RuntimeError(f"Impossible d'ouvrir {port}")
        if not self.ph.setBaudRate(baudrate):
            raise RuntimeError(f"Erreur baudrate {baudrate}")

        self._set_latency_timer(port, 1)

        print(f'[DXL v3.0] {port} @ {baudrate} baud | '
              f'servos={self.servo_ids} | {read_hz}Hz | 4 moteurs A+B')
        print(f'[DXL] CT_GOAL_VEL_B=108  CT_NOW_VEL_B=132  '
              f'READ {READ_START}→{READ_END} ({READ_LEN}B)')

        self._startup_ping()
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True,
                         name='dxl-read').start()

    def _set_latency_timer(self, port: str, ms: int = 1):
        try:
            dev = port.split('/')[-1]
            candidates = [
                f'/sys/bus/usb-serial/devices/{dev}/latency_timer',
                f'/sys/bus/usb/drivers/ftdi_sio/{dev}/latency_timer',
            ]
            for lt in candidates:
                try:
                    subprocess.run(['sudo', 'tee', lt],
                                   input=str(ms).encode(),
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   timeout=2)
                    with open(lt) as f:
                        val = f.read().strip()
                    print(f'[DXL] Latency timer: {val}ms')
                    return
                except FileNotFoundError:
                    continue
        except Exception as e:
            print(f'[DXL] Latency timer: erreur ({e})')

    def _sim_loop(self):
        while self._running:
            for s in self.servos.values():
                s.last_seen          = time.time()
                s.present_velocity   = s.goal_velocity
                s.present_velocity_b = s.goal_velocity_b
            time.sleep(1.0 / self.read_hz)

    def _startup_ping(self):
        print('[DXL] Ping initial...')
        for sid in self.servo_ids:
            model, result, _ = self.pkt.ping(self.ph, sid)
            if result == COMM_SUCCESS:
                print(f'  S{sid}: OK model=0x{model:04X}')
            else:
                print(f'  S{sid}: ÉCHEC {self.pkt.getTxRxResult(result)}')
            time.sleep(0.05)

    # ────────────────────────────────────────
    # Boucle de lecture [FIX CRIT-1+3]
    # ────────────────────────────────────────
    def _read_loop(self):
        interval = 1.0 / self.read_hz
        while self._running:
            t0 = time.time()
            with self.lock:
                for i, sid in enumerate(self.servo_ids):
                    if not self._running:
                        break
                    if i > 0:
                        time.sleep(self.INTER_SERVO_MS)
                    self._read_one_nolock(sid)
            elapsed = time.time() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    def _read_one_nolock(self, sid: int):
        data, result, _ = self.pkt.readTxRx(
            self.ph, sid, READ_START, READ_LEN)
        if result != COMM_SUCCESS:
            self.servos[sid].read_errors += 1
            ec = self.servos[sid].read_errors
            if ec == 1 or ec % 20 == 0:
                print(f'[DXL ERR] S{sid} ({ec}x): '
                      f'{self.pkt.getTxRxResult(result)}')
            return
        if data and len(data) >= READ_LEN:
            self.servos[sid].update_from_raw(bytes(data[:READ_LEN]))

    # ────────────────────────────────────────
    # Primitives d'écriture
    # ────────────────────────────────────────
    def _w1(self, sid: int, addr: int, val: int) -> bool:
        if self._sim: return True
        with self.lock:
            r, _ = self.pkt.write1ByteTxRx(self.ph, sid, addr, val & 0xFF)
        ok = (r == COMM_SUCCESS)
        if not ok:
            print(f'[DXL] w1 S{sid}@{addr}: {self.pkt.getTxRxResult(r)}')
        return ok

    def _w2(self, sid: int, addr: int, val: int) -> bool:
        if self._sim: return True
        with self.lock:
            r, _ = self.pkt.write2ByteTxRx(
                self.ph, sid, addr, val & 0xFFFF)
        return (r == COMM_SUCCESS)

    def _w4(self, sid: int, addr: int, val: int) -> bool:
        if self._sim: return True
        with self.lock:
            r, _ = self.pkt.write4ByteTxRx(
                self.ph, sid, addr, val & 0xFFFFFFFF)
        return (r == COMM_SUCCESS)

    # ────────────────────────────────────────
    # API publique
    # ────────────────────────────────────────
    def set_operating_mode(self, sid: int, mode: int) -> bool:
        """[FIX CRIT-2] commanded_mode mis à jour immédiatement."""
        state = self.servos.get(sid)
        if state is None:
            return False
        was_torque = state.torque_commanded
        if was_torque:
            self._w1(sid, CT.TORQUE_ENABLE, 0)
            state.torque_commanded = False
            time.sleep(0.12)
        ok = self._w1(sid, CT.OPERATING_MODE, mode)
        state.commanded_mode = mode
        state.operating_mode = mode
        time.sleep(0.12)
        if was_torque:
            self._w1(sid, CT.TORQUE_ENABLE, 1)
            state.torque_commanded = True
        print(f'[DXL] S{sid} mode→{mode} {"OK" if ok else "ERR"}')
        return ok

    def set_torque(self, sid: int, enable: bool) -> bool:
        ok = self._w1(sid, CT.TORQUE_ENABLE, 1 if enable else 0)
        state = self.servos.get(sid)
        if state is not None:
            state.torque_commanded = enable
            if ok:
                state.torque_enabled = enable
        return ok

    def set_profile(self, sid: int, velocity_rpm: float = 0.0) -> bool:
        """
        [FIX CT-2] Profile velocity seulement (registre 112).
        L'ESP32 n'a PAS de registre profile_acceleration —
        la rampe est gérée en interne (RAMP_RPM_S=800 dans le firmware).
        """
        v = abs(Conv.rpm_to_vel(velocity_rpm)) if velocity_rpm > 0 else 0
        ok = self._w4(sid, CT.PROFILE_VELOCITY, v)
        state = self.servos.get(sid)
        if ok and state is not None:
            state.profile_velocity = velocity_rpm
        return ok

    def goal_velocity(self, sid: int, rpm: float) -> bool:
        """Moteur A (avant) — CT_GOAL_VEL (104)."""
        raw = Conv.rpm_to_vel(rpm)
        if raw < 0:
            raw += 0x100000000
        ok = self._w4(sid, CT.GOAL_VELOCITY, raw & 0xFFFFFFFF)
        if ok:
            self.servos[sid].goal_velocity = rpm
        return ok

    def goal_velocity_b(self, sid: int, rpm: float) -> bool:
        """
        Moteur B (arrière) — CT_GOAL_VEL_B (108).
        [4M-1] Nouvelle méthode — était incorrectement PROFILE_ACCELERATION.
        """
        raw = Conv.rpm_to_vel(rpm)
        if raw < 0:
            raw += 0x100000000
        ok = self._w4(sid, CT.GOAL_VELOCITY_B, raw & 0xFFFFFFFF)
        if ok:
            self.servos[sid].goal_velocity_b = rpm
        return ok

    def goal_position(self, sid: int, degrees: float) -> bool:
        raw = Conv.deg_to_pos(degrees) & 0xFFFFFFFF
        ok  = self._w4(sid, CT.GOAL_POSITION, raw)
        if ok:
            self.servos[sid].goal_position = degrees
        return ok

    def goal_current(self, sid: int, ma: float) -> bool:
        raw = Conv.ma_to_cur(ma)
        if raw < 0:
            raw += 0x10000
        ok = self._w2(sid, CT.GOAL_CURRENT, raw & 0xFFFF)
        if ok:
            self.servos[sid].goal_current = ma
        return ok

    def goal_pwm(self, sid: int, pct: float) -> bool:
        raw = Conv.pct_to_pwm(pct)
        if raw < 0:
            raw += 0x10000
        ok = self._w2(sid, CT.GOAL_PWM, raw & 0xFFFF)
        if ok:
            self.servos[sid].goal_pwm = pct
        return ok

    def sync_goal_velocity(self, targets: dict) -> bool:
        """
        SyncWrite moteur A sur plusieurs slaves simultanément.
        targets = {sid: rpm_a, ...}
        """
        if self._sim:
            for sid, rpm in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_velocity = rpm
            return True
        sw = GroupSyncWrite(self.ph, self.pkt, CT.GOAL_VELOCITY, 4)
        for sid, rpm in targets.items():
            raw = Conv.rpm_to_vel(rpm)
            if raw < 0: raw += 0x100000000
            raw &= 0xFFFFFFFF
            sw.addParam(sid, [
                DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)),
                DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw)),
            ])
        with self.lock:
            result = sw.txPacket()
        ok = (result == COMM_SUCCESS)
        if ok:
            for sid, rpm in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_velocity = rpm
        return ok

    def sync_goal_velocity_b(self, targets: dict) -> bool:
        """
        SyncWrite moteur B sur plusieurs slaves simultanément.
        targets = {sid: rpm_b, ...}
        [4M-1] Nouvelle méthode pour moteur B (CT_GOAL_VEL_B = 108).
        """
        if self._sim:
            for sid, rpm in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_velocity_b = rpm
            return True
        sw = GroupSyncWrite(self.ph, self.pkt, CT.GOAL_VELOCITY_B, 4)
        for sid, rpm in targets.items():
            raw = Conv.rpm_to_vel(rpm)
            if raw < 0: raw += 0x100000000
            raw &= 0xFFFFFFFF
            sw.addParam(sid, [
                DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)),
                DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw)),
            ])
        with self.lock:
            result = sw.txPacket()
        ok = (result == COMM_SUCCESS)
        if ok:
            for sid, rpm in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_velocity_b = rpm
        return ok

    def sync_goal_position(self, targets: dict) -> bool:
        if self._sim:
            for sid, d in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_position = d
            return True
        sw = GroupSyncWrite(self.ph, self.pkt, CT.GOAL_POSITION, 4)
        for sid, deg in targets.items():
            raw = Conv.deg_to_pos(deg) & 0xFFFFFFFF
            sw.addParam(sid, [
                DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)),
                DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw)),
            ])
        with self.lock:
            result = sw.txPacket()
        ok = (result == COMM_SUCCESS)
        if ok:
            for sid, deg in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_position = deg
        return ok

    def sync_torque(self, ids: list, enable: bool) -> bool:
        if self._sim:
            for sid in ids:
                if sid in self.servos:
                    self.servos[sid].torque_enabled   = enable
                    self.servos[sid].torque_commanded = enable
            return True
        sw = GroupSyncWrite(self.ph, self.pkt, CT.TORQUE_ENABLE, 1)
        for sid in ids:
            sw.addParam(sid, [1 if enable else 0])
        with self.lock:
            result = sw.txPacket()
        ok = (result == COMM_SUCCESS)
        if ok:
            for sid in ids:
                if sid in self.servos:
                    self.servos[sid].torque_enabled   = enable
                    self.servos[sid].torque_commanded = enable
        return ok

    def emergency_stop(self, ids: list = None):
        ids = ids or list(self.servos.keys())
        print(f'[DXL] EMERGENCY STOP {ids}')
        self.sync_torque(ids, False)

    def close(self):
        self._running = False
        time.sleep(0.35)
        if not self._sim:
            try:
                self.ph.closePort()
            except Exception:
                pass
        print('[DXL] Port fermé')


# ════════════════════════════════════════════
# SERVO — façade simplifiée (4 moteurs)
# ════════════════════════════════════════════
class Servo:
    def __init__(self, bus: DynamixelBus, sid: int):
        self.bus   = bus
        self.id    = sid
        self.state = bus.servos[sid]

    def set_mode(self, mode: int) -> bool:
        return self.bus.set_operating_mode(self.id, mode)

    def enable_torque(self) -> bool:
        return self.bus.set_torque(self.id, True)

    def disable_torque(self) -> bool:
        return self.bus.set_torque(self.id, False)

    def set_profile(self, v: float = 0.0, a: float = 0.0) -> bool:
        """[FIX CT-2] a (acceleration) ignoré — géré par l'ESP32 en interne."""
        return self.bus.set_profile(self.id, velocity_rpm=v)

    def velocity(self, rpm: float) -> bool:
        """Moteur A (avant) — CT_GOAL_VEL (104)."""
        self.state.goal_velocity = rpm
        return self.bus.goal_velocity(self.id, rpm)

    def velocity_b(self, rpm: float) -> bool:
        """
        Moteur B (arrière) — CT_GOAL_VEL_B (108).
        [4M-3] Nouvelle méthode.
        """
        self.state.goal_velocity_b = rpm
        return self.bus.goal_velocity_b(self.id, rpm)

    def stop(self) -> bool:
        """Arrête les deux moteurs A et B."""
        self.state.goal_velocity   = 0.0
        self.state.goal_velocity_b = 0.0
        ok  = self.bus.goal_velocity(self.id, 0.0)
        ok &= self.bus.goal_velocity_b(self.id, 0.0)
        return bool(ok)

    def move_to(self, degrees: float,
                profile_rpm: float = None,
                profile_accel: float = None) -> bool:
        if profile_rpm is not None:
            self.set_profile(v=profile_rpm)
        self.state.goal_position = degrees
        return self.bus.goal_position(self.id, degrees)

    def move_to_with_current(self, degrees: float, max_ma: float,
                              profile_rpm: float = None) -> bool:
        self.bus.goal_current(self.id, max_ma)
        if profile_rpm is not None:
            self.set_profile(v=profile_rpm)
        self.state.goal_position = degrees
        return self.bus.goal_position(self.id, degrees)

    def pwm(self, pct: float) -> bool:
        self.state.goal_pwm = pct
        return self.bus.goal_pwm(self.id, pct)

    def is_moving(self, thr: float = 1.0) -> bool:
        return (abs(self.state.present_velocity)   > thr or
                abs(self.state.present_velocity_b) > thr)

    def has_error(self) -> bool:
        return self.state.hardware_error != 0

    def status_str(self) -> str:
        s  = self.state
        ok = 'OK' if s.is_alive() else 'XX'
        stall = ' [STALL]' if s.stall_flag else ''
        return (
            f'S{self.id}[{ok}] '
            f'mode={s.commanded_mode} torq={"ON" if s.torque_commanded else "OFF"} | '
            f'A: SP={s.goal_velocity:+6.1f}  mes={s.present_velocity:+6.1f}rpm  '
            f'cur={s.present_current:5.0f}mA | '
            f'B: SP={s.goal_velocity_b:+6.1f}  mes={s.present_velocity_b:+6.1f}rpm | '
            f'zone={s.vel_zone}{stall}  errs={s.read_errors}  {s.push_rate:.1f}Hz'
        )


# ════════════════════════════════════════════
# KEEPALIVE — renvoie les consignes périodiquement
# [4M-4] Renvoie aussi goal_velocity_b
# ════════════════════════════════════════════
class KeepAlive:
    def __init__(self, bus: DynamixelBus, servos: list, interval: float = 1.0):
        self.bus      = bus
        self.servos   = servos
        self.interval = interval
        self._running = False
        self._thread  = None

    def start(self):
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, daemon=True, name='dxl-keepalive')
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            time.sleep(self.interval)
            for s in self.servos:
                st = s.state
                if not st.torque_commanded:
                    continue
                if st.commanded_mode == Mode.VELOCITY:
                    # [4M-4] Renvoyer moteur A ET moteur B
                    if st.goal_velocity != 0.0:
                        self.bus.goal_velocity(s.id, st.goal_velocity)
                    if st.goal_velocity_b != 0.0:
                        self.bus.goal_velocity_b(s.id, st.goal_velocity_b)


# ════════════════════════════════════════════
# SCRIPT AUTONOME (diagnostic)
# ════════════════════════════════════════════
if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',     default='/dev/ttyTHS1')
    ap.add_argument('--baudrate', default=115200, type=int)
    ap.add_argument('--ids',      default='1,2')
    ap.add_argument('--hz',       default=25.0, type=float)
    ap.add_argument('--demo',     action='store_true')
    args = ap.parse_args()

    ids    = [int(x) for x in args.ids.split(',')]
    bus    = DynamixelBus(port=args.port, baudrate=args.baudrate,
                          servo_ids=ids, read_hz=args.hz)
    servos = [Servo(bus, sid) for sid in ids]
    ka     = KeepAlive(bus, servos)
    ka.start()

    if args.demo:
        s = servos[0]
        print(f'[DEMO] S{s.id}: moteur A=+50rpm  moteur B=+50rpm')
        s.set_mode(Mode.VELOCITY)
        s.enable_torque()
        s.velocity(50.0)
        s.velocity_b(50.0)

    print('[MONITOR] Ctrl+C pour arrêter')
    try:
        while True:
            for s in servos:
                print('  ' + s.status_str())
            time.sleep(0.25)
    except KeyboardInterrupt:
        ka.stop()
        for s in servos:
            s.stop()
            s.disable_torque()
        bus.close()

#!/usr/bin/env python3
# ============================================
# DYNAMIXEL DRIVER v2.5 — ESP32 daisy-chain / Python 3.6+
#
# CORRECTIONS v2.5 — corrections des bugs critiques identifiés
#   dans les logs de débogage:
#
#   [FIX CRIT-1] Suppression de GroupBulkRead → retour lectures
#       séquentielles individuelles
#
#       PROBLÈME: GroupBulkRead.txRxPacket() attend les réponses de
#       TOUS les servos dans la même fenêtre temporelle. Avec deux
#       ESP32 daisy-chained sur RS485 qui répondent séquentiellement
#       (délai firmware: 500µs + mpos×4000µs), le SDK timeout avant
#       d'avoir reçu la réponse du second ESP32.
#       Symptôme: "BulkRead ERREUR: There is no status packet!" même
#       quand le PING fonctionne.
#
#       SOLUTION: lecture séquentielle readTxRx() individuelle par servo,
#       avec seulement 20ms entre les deux lectures (pas 150ms).
#       Latence totale: ~25ms pour 2 servos → feedback à 25Hz réel.
#
#   [FIX CRIT-2] Mode commandé tracké séparément de l'état lu
#
#       PROBLÈME: si les lectures échouent (état non mis à jour),
#       servo.state.operating_mode reste à la valeur initiale (3 =
#       POSITION). Le nœud ROS2 compare "mode demandé (1)" vs "mode
#       lu (3 figé)" → détecte un changement à chaque callback →
#       envoie set_mode() à 3Hz → sature le bus → les lectures
#       continuent d'échouer → boucle infinie.
#       Symptôme dans les logs: "S1 changement de mode 3 → 1" répété
#       toutes les 270ms indéfiniment.
#
#       SOLUTION: ServoState.commanded_mode = mode effectivement envoyé
#       au servo (indépendant de ce qui est lu). Le nœud compare
#       mode_demandé vs commanded_mode (pas vs present_mode).
#       set_mode() met à jour commanded_mode immédiatement même si
#       la confirmation n'est pas lue en retour.
#
#   [FIX CRIT-3] Bus mutex étendu à toutes les opérations série
#
#       PROBLÈME: la boucle de lecture tourne dans un thread daemon
#       sans prendre le lock pendant le delayMicroseconds entre
#       servos, laissant une fenêtre où un write() peut interrompre
#       une transaction de lecture en cours.
#
#       SOLUTION: le lock est maintenu pour TOUTE la séquence de
#       lecture multi-servo (pas relâché entre les deux reads).
#       Les writes attendent que la séquence de lecture soit terminée.
#
#   [FIX CRIT-4] Timeout ServoState réduit à 2.0s
#       À 25Hz, 2s = 50 paquets perdus consécutifs. En dessous on
#       ne déclare pas un servo mort trop vite.
#
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
# REGISTRES CONTROL TABLE (ESP32 v1.9)
# ════════════════════════════════════════════
class CT:
    OPERATING_MODE          = 11
    TORQUE_ENABLE           = 64
    HARDWARE_ERROR_STATUS   = 70
    GOAL_PWM                = 100
    GOAL_CURRENT            = 102
    GOAL_VELOCITY           = 104
    PROFILE_ACCELERATION    = 108
    PROFILE_VELOCITY        = 112
    GOAL_POSITION           = 116
    PRESENT_PWM             = 124
    PRESENT_CURRENT         = 126
    PRESENT_VELOCITY        = 128
    PRESENT_POSITION        = 132
    PRESENT_VOLTAGE         = 144
    PRESENT_TEMPERATURE     = 146
    VEL_ZONE                = 156   # [v1.9] Zone PID (0/1/2), lecture seule
    STALL_FLAG              = 157   # [v1.9] Calage détecté (0/1), lecture seule


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
# CONVERSIONS UNITÉS DYNAMIXEL (X-series)
# ════════════════════════════════════════════
class Conv:
    POS_UNIT  = 360.0 / 4096.0   # deg/LSB
    VEL_UNIT  = 0.229              # rpm/LSB
    CUR_UNIT  = 2.69               # mA/LSB
    PWM_UNIT  = 0.113              # %/LSB
    VOLT_UNIT = 0.1                # V/LSB

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
# PLAGE DE LECTURE COMMUNE
# ════════════════════════════════════════════
# On lit depuis CT_TORQUE_ENABLE (64) jusqu'à CT_STALL_FLAG (157).
# Un seul readTxRx() de 94 octets couvre tous les registres utiles.
READ_START = CT.TORQUE_ENABLE         # 64
READ_END   = CT.STALL_FLAG           # 157
READ_LEN   = READ_END - READ_START + 1  # 94 octets

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
# ÉTAT D'UN SERVO
# ════════════════════════════════════════════
@dataclass
class ServoState:
    servo_id: int

    # [FIX CRIT-2] Mode commandé (ce qu'on a ENVOYÉ) vs mode présent (ce qu'on LIT)
    # La comparaison pour les changements de mode doit utiliser commanded_mode,
    # pas operating_mode (qui peut rester figé si les lectures échouent).
    commanded_mode:  int   = Mode.POSITION    # Mode envoyé au servo
    operating_mode:  int   = Mode.POSITION    # Mode lu (peut être obsolète)
    torque_enabled:  bool  = False
    torque_commanded: bool = False             # Torque qu'on a commandé

    present_position:    float = 0.0
    present_velocity:    float = 0.0   # RPM
    present_current:     float = 0.0   # mA
    present_pwm:         float = 0.0   # %
    present_voltage:     float = 0.0   # V
    present_temperature: int   = 0

    goal_position:       float = 0.0
    goal_velocity:       float = 0.0
    goal_current:        float = 0.0
    goal_pwm:            float = 0.0

    profile_velocity:     float = 0.0
    profile_acceleration: float = 0.0

    hardware_error: int  = 0
    vel_zone:       int  = 1
    stall_flag:     bool = False

    # Statistiques
    last_seen:    float = 0.0
    push_count:   int   = 0
    push_rate:    float = 0.0
    read_errors:  int   = 0

    hist_current:  List[float] = field(default_factory=lambda: [0.0] * HISTORY_SIZE)
    hist_velocity: List[float] = field(default_factory=lambda: [0.0] * HISTORY_SIZE)

    _rate_t: float = field(default_factory=time.time, repr=False)
    _rate_n: int   = field(default=0, repr=False)

    # [FIX CRIT-4] Timeout 2.0s (pas 1.0s ni 0.5s)
    TIMEOUT_S: float = 2.0

    def update_from_raw(self, raw: bytes):
        def g(a, l): return _extract(raw, READ_START, a, l)

        te  = g(CT.TORQUE_ENABLE,         1)
        hw  = g(CT.HARDWARE_ERROR_STATUS,  1)
        pwm = g(CT.PRESENT_PWM,           2)
        cur = g(CT.PRESENT_CURRENT,       2)
        vel = g(CT.PRESENT_VELOCITY,      4)
        pos = g(CT.PRESENT_POSITION,      4)
        vlt = g(CT.PRESENT_VOLTAGE,       2)
        tmp = g(CT.PRESENT_TEMPERATURE,   1)
        vzn = g(CT.VEL_ZONE,              1)
        stl = g(CT.STALL_FLAG,            1)

        if te  is not None: self.torque_enabled      = bool(te)
        if hw  is not None: self.hardware_error      = hw
        if pwm is not None: self.present_pwm         = Conv.pwm_to_pct(_u2s16(pwm))
        if cur is not None: self.present_current     = Conv.cur_to_ma(_u2s16(cur))
        if vel is not None: self.present_velocity    = Conv.vel_to_rpm(_u2s32(vel))
        if pos is not None: self.present_position    = Conv.pos_to_deg(_u2s32(pos))
        if vlt is not None: self.present_voltage     = vlt * Conv.VOLT_UNIT
        if tmp is not None: self.present_temperature = tmp
        if vzn is not None: self.vel_zone            = vzn
        if stl is not None: self.stall_flag          = bool(stl)
        # Note: on ne met PAS à jour operating_mode depuis la lecture,
        # car le registre CT_OPERATING_MODE (11) est en dehors de READ_START (64).
        # On fait confiance à commanded_mode pour la logique de contrôle.

        self.last_seen   = time.time()
        self.push_count += 1
        self.read_errors = 0    # Reset compteur d'erreurs consécutives
        self._rate_n    += 1
        dt = time.time() - self._rate_t
        if dt >= 1.0:
            self.push_rate = self._rate_n / dt
            self._rate_n   = 0
            self._rate_t   = time.time()

        self.hist_current  = self.hist_current[1:]  + [self.present_current]
        self.hist_velocity = self.hist_velocity[1:] + [self.present_velocity]

    def is_alive(self) -> bool:
        return self.last_seen > 0 and (time.time() - self.last_seen) < self.TIMEOUT_S


# ════════════════════════════════════════════
# BUS DYNAMIXEL
# ════════════════════════════════════════════
class DynamixelBus:
    PROTOCOL       = 2.0
    # [FIX CRIT-1] 20ms entre les deux lectures séquentielles
    # (suffisant pour que l'ESP32 1 termine sa réponse avant qu'on lise l'ESP32 2)
    INTER_SERVO_MS = 0.020   # 20ms

    def __init__(self, port: str = '/dev/ttyUSB0',
                 baudrate: int = 115200,
                 servo_ids: list = None,
                 read_hz: float = 25.0):
        self.port      = port
        self.baudrate  = baudrate
        self.servo_ids = servo_ids or [1]
        self.read_hz   = read_hz
        # [FIX CRIT-3] Lock unique pour TOUTE la séquence de lecture multi-servo
        self.lock      = threading.Lock()
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

        # Tentative de réduction du latency timer USB (1ms au lieu de 16ms)
        self._set_latency_timer(port, 1)

        print(f'[DXL] {port} @ {baudrate} baud | servos={self.servo_ids} '
              f'| {read_hz}Hz | inter_servo={int(self.INTER_SERVO_MS*1000)}ms')

        self._startup_ping()
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True, name='dxl-read').start()

    def _set_latency_timer(self, port: str, ms: int = 1):
        try:
            dev = port.split('/')[-1]
            # Chercher dans différents emplacements selon le kernel
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
                    print(f'[DXL] Latency timer: {val}ms ({lt})')
                    return
                except FileNotFoundError:
                    continue
            print('[DXL] Latency timer: chemin non trouvé (non critique)')
        except Exception as e:
            print(f'[DXL] Latency timer: erreur ({e})')

    def _sim_loop(self):
        while self._running:
            for s in self.servos.values():
                s.last_seen = time.time()
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
    # [FIX CRIT-1+3] Boucle de lecture séquentielle avec lock global
    # ────────────────────────────────────────
    def _read_loop(self):
        interval = 1.0 / self.read_hz
        while self._running:
            t0 = time.time()
            # [FIX CRIT-3] Lock maintenu pour TOUTE la séquence multi-servo.
            # Garantit qu'aucun write ne peut s'intercaler entre les deux reads.
            with self.lock:
                for i, sid in enumerate(self.servo_ids):
                    if not self._running:
                        break
                    if i > 0:
                        # Petite pause entre servos (dans le lock = bus occupé)
                        time.sleep(self.INTER_SERVO_MS)
                    self._read_one_nolock(sid)
            elapsed = time.time() - t0
            rem = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    def _read_one_nolock(self, sid: int):
        """Lecture sans prendre le lock (doit être appelé avec lock déjà acquis)."""
        data, result, _ = self.pkt.readTxRx(self.ph, sid, READ_START, READ_LEN)
        if result != COMM_SUCCESS:
            self.servos[sid].read_errors += 1
            err_count = self.servos[sid].read_errors
            # Log seulement tous les 20 échecs pour ne pas saturer les logs
            if err_count == 1 or err_count % 20 == 0:
                print(f'[DXL ERR] S{sid} ({err_count}x): '
                      f'{self.pkt.getTxRxResult(result)}')
            return
        if data and len(data) >= READ_LEN:
            self.servos[sid].update_from_raw(bytes(data[:READ_LEN]))

    # ────────────────────────────────────────
    # Écritures (prennent le lock)
    # ────────────────────────────────────────
    def _w1(self, sid: int, addr: int, val: int) -> bool:
        if self._sim:
            return True
        with self.lock:
            r, _ = self.pkt.write1ByteTxRx(self.ph, sid, addr, val & 0xFF)
        ok = (r == COMM_SUCCESS)
        if not ok:
            print(f'[DXL] w1 S{sid}@{addr}: {self.pkt.getTxRxResult(r)}')
        return ok

    def _w2(self, sid: int, addr: int, val: int) -> bool:
        if self._sim:
            return True
        with self.lock:
            r, _ = self.pkt.write2ByteTxRx(self.ph, sid, addr, val & 0xFFFF)
        return (r == COMM_SUCCESS)

    def _w4(self, sid: int, addr: int, val: int) -> bool:
        if self._sim:
            return True
        with self.lock:
            r, _ = self.pkt.write4ByteTxRx(self.ph, sid, addr, val & 0xFFFFFFFF)
        return (r == COMM_SUCCESS)

    # ────────────────────────────────────────
    # API publique
    # ────────────────────────────────────────
    def set_operating_mode(self, sid: int, mode: int) -> bool:
        """
        [FIX CRIT-2] Met à jour commanded_mode IMMÉDIATEMENT,
        indépendamment du succès de la communication.
        Garantit que le nœud ROS2 ne redétecte pas un "changement
        de mode" si la lecture est momentanément indisponible.
        """
        state = self.servos.get(sid)
        if state is None:
            return False

        was_torque = state.torque_commanded
        if was_torque:
            self._w1(sid, CT.TORQUE_ENABLE, 0)
            state.torque_commanded = False
            time.sleep(0.12)

        ok = self._w1(sid, CT.OPERATING_MODE, mode)

        # [FIX CRIT-2] Mettre à jour commanded_mode même si ok=False
        # (l'ESP32 a peut-être quand même appliqué la commande sans ACK)
        state.commanded_mode  = mode
        state.operating_mode  = mode   # Synchroniser aussi operating_mode

        time.sleep(0.12)
        if was_torque:
            self._w1(sid, CT.TORQUE_ENABLE, 1)
            state.torque_commanded = True

        print(f'[DXL] S{sid} mode→{mode} {"OK" if ok else "ERR (commandé quand même)"}')
        return ok

    def set_torque(self, sid: int, enable: bool) -> bool:
        ok = self._w1(sid, CT.TORQUE_ENABLE, 1 if enable else 0)
        state = self.servos.get(sid)
        if state is not None:
            # [FIX CRIT-2] Mettre à jour l'état commandé immédiatement
            state.torque_commanded = enable
            if ok:
                state.torque_enabled = enable
        return ok

    def set_profile(self, sid: int,
                    velocity_rpm: float = 0.0,
                    accel_rpm2: float   = 0.0) -> bool:
        v = abs(Conv.rpm_to_vel(velocity_rpm)) if velocity_rpm > 0 else 0
        a = int(abs(accel_rpm2) / 214.577)     if accel_rpm2  > 0 else 0
        ok  = self._w4(sid, CT.PROFILE_VELOCITY,     v)
        ok &= self._w4(sid, CT.PROFILE_ACCELERATION, a)
        state = self.servos.get(sid)
        if ok and state is not None:
            state.profile_velocity     = velocity_rpm
            state.profile_acceleration = accel_rpm2
        return bool(ok)

    def goal_position(self, sid: int, degrees: float) -> bool:
        raw = Conv.deg_to_pos(degrees) & 0xFFFFFFFF
        ok  = self._w4(sid, CT.GOAL_POSITION, raw)
        if ok:
            self.servos[sid].goal_position = degrees
        return ok

    def goal_velocity(self, sid: int, rpm: float) -> bool:
        raw = Conv.rpm_to_vel(rpm)
        if raw < 0:
            raw += 0x100000000
        ok = self._w4(sid, CT.GOAL_VELOCITY, raw & 0xFFFFFFFF)
        if ok:
            self.servos[sid].goal_velocity = rpm
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
        SyncWrite: envoie les consignes RPM à plusieurs servos dans
        un seul paquet — les deux moteurs démarrent simultanément,
        éliminant le décalage temporel inter-moteur.
        """
        if self._sim:
            for sid, rpm in targets.items():
                if sid in self.servos:
                    self.servos[sid].goal_velocity = rpm
            return True

        sw = GroupSyncWrite(self.ph, self.pkt, CT.GOAL_VELOCITY, 4)
        for sid, rpm in targets.items():
            raw = Conv.rpm_to_vel(rpm)
            if raw < 0:
                raw += 0x100000000
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
# SERVO (façade simplifiée)
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
        return self.bus.set_profile(self.id, velocity_rpm=v, accel_rpm2=a)

    def velocity(self, rpm: float) -> bool:
        self.state.goal_velocity = rpm
        return self.bus.goal_velocity(self.id, rpm)

    def stop(self) -> bool:
        self.state.goal_velocity = 0.0
        return self.bus.goal_velocity(self.id, 0.0)

    def move_to(self, degrees: float,
                profile_rpm: float = None,
                profile_accel: float = None) -> bool:
        if profile_rpm is not None or profile_accel is not None:
            self.set_profile(
                v=profile_rpm   if profile_rpm   is not None else self.state.profile_velocity,
                a=profile_accel if profile_accel is not None else self.state.profile_acceleration,
            )
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
        return abs(self.state.present_velocity) > thr

    def is_at_target(self, tol: float = 1.0) -> bool:
        return abs(self.state.present_position - self.state.goal_position) < tol

    def has_error(self) -> bool:
        return self.state.hardware_error != 0

    def status_str(self) -> str:
        s  = self.state
        ok = 'OK' if s.is_alive() else 'XX'
        stall = ' [STALL]' if s.stall_flag else ''
        return (
            f'S{self.id}[{ok}] '
            f'cmd_mode={s.commanded_mode} torq={"ON" if s.torque_commanded else "OFF"} '
            f'pos={s.present_position:7.2f}deg '
            f'vel={s.present_velocity:+6.1f}rpm '
            f'cur={s.present_current:6.1f}mA '
            f'zone={s.vel_zone}{stall} '
            f'errs={s.read_errors} rate={s.push_rate:.1f}Hz'
        )


# ════════════════════════════════════════════
# KEEPALIVE — renvoie les consignes périodiquement
# ════════════════════════════════════════════
class KeepAlive:
    """Renvoie périodiquement la consigne de vitesse pour éviter le timeout ESP32."""

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
                if st.commanded_mode == Mode.VELOCITY and st.goal_velocity != 0.0:
                    self.bus.goal_velocity(s.id, st.goal_velocity)


# ════════════════════════════════════════════
# SCRIPT AUTONOME (diagnostic)
# ════════════════════════════════════════════
if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',     default='/dev/ttyUSB0')
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
        print(f'[DEMO] S{s.id}: VELOCITY 30rpm (rampe douce)')
        s.set_mode(Mode.VELOCITY)
        s.enable_torque()
        s.velocity(30.0)

    print('[MONITOR] Ctrl+C pour arrêter')
    try:
        while True:
            for s in servos:
                print('  ' + s.status_str())
            time.sleep(0.2)
    except KeyboardInterrupt:
        ka.stop()
        for s in servos:
            s.disable_torque()
        bus.close()

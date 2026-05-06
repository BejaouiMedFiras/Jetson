#!/usr/bin/env python3
"""
rs485_diag_v2.py — Diagnostic RS485 complet pour ESP32 Dynamixel slave
Teste plusieurs baudrates, plusieurs IDs, et affiche les octets bruts reçus.

Usage:
    python3 rs485_diag_v2.py
    python3 rs485_diag_v2.py --port /dev/ttyUSB1
    python3 rs485_diag_v2.py --port /dev/ttyUSB0 --baudrate 57600
"""

import sys
import time
import struct
import argparse

try:
    import serial
except ImportError:
    print("Installer pyserial: pip3 install pyserial")
    sys.exit(1)


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x8005 if (crc & 0x8000) else crc << 1
        crc &= 0xFFFF
    return crc


def ping_packet(servo_id: int) -> bytes:
    """Construit un paquet PING Dynamixel Protocol 2.0"""
    header = bytes([0xFF, 0xFF, 0xFD, 0x00, servo_id, 0x03, 0x00, 0x01])
    crc = crc16(header)
    return header + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_packet(servo_id: int, addr: int, length: int) -> bytes:
    """Construit un paquet READ Dynamixel Protocol 2.0"""
    params = bytes([addr & 0xFF, (addr >> 8) & 0xFF,
                    length & 0xFF, (length >> 8) & 0xFF])
    header = bytes([0xFF, 0xFF, 0xFD, 0x00, servo_id,
                    (len(params) + 3) & 0xFF, ((len(params) + 3) >> 8) & 0xFF,
                    0x02])
    full = header + params
    crc = crc16(full)
    return full + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def hex_str(data: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in data)


def test_ping(ser: serial.Serial, servo_id: int, timeout: float = 0.5) -> bool:
    pkt = ping_packet(servo_id)
    ser.reset_input_buffer()

    # Vider d'abord tout ce qui traîne sur le bus
    time.sleep(0.05)
    ser.reset_input_buffer()

    print(f"  TX PING→S{servo_id} ({len(pkt)} octets): {hex_str(pkt)}")
    ser.write(pkt)
    ser.flush()

    t0 = time.time()
    resp = bytearray()
    while (time.time() - t0) < timeout:
        chunk = ser.read(64)
        if chunk:
            resp.extend(chunk)
            if len(resp) >= 14:
                break
        time.sleep(0.001)

    if not resp:
        print(f"  RX: SILENCE TOTAL ({timeout*1000:.0f}ms)")
        return False

    print(f"  RX ({len(resp)} octets): {hex_str(bytes(resp))}")

    # Analyser la réponse
    # Chercher l'en-tête FF FF FD 00
    for i in range(len(resp) - 3):
        if resp[i:i+4] == bytes([0xFF, 0xFF, 0xFD, 0x00]):
            if i + 9 <= len(resp):
                rid  = resp[i+4]
                rlen = resp[i+5] | (resp[i+6] << 8)
                inst = resp[i+7]
                err  = resp[i+8]
                if inst == 0x55:  # STATUS packet
                    model_l = resp[i+9]  if i+9  < len(resp) else 0
                    model_h = resp[i+10] if i+10 < len(resp) else 0
                    model   = model_l | (model_h << 8)
                    print(f"  ✓ STATUS reçu: ID={rid} model=0x{model:04X} err={err}")
                    return True
    print(f"  ⚠ Données reçues mais format inattendu")
    return False


def test_read(ser: serial.Serial, servo_id: int, addr: int = 132, length: int = 4,
              timeout: float = 0.5) -> bool:
    """Test READ sur l'adresse position (132)"""
    pkt = read_packet(servo_id, addr, length)
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.reset_input_buffer()

    print(f"  TX READ@{addr}→S{servo_id} ({len(pkt)} octets): {hex_str(pkt)}")
    ser.write(pkt)
    ser.flush()

    t0 = time.time()
    resp = bytearray()
    while (time.time() - t0) < timeout:
        chunk = ser.read(64)
        if chunk:
            resp.extend(chunk)
            if len(resp) >= 13 + length:
                break
        time.sleep(0.001)

    if not resp:
        print(f"  RX: SILENCE ({timeout*1000:.0f}ms)")
        return False

    print(f"  RX ({len(resp)} octets): {hex_str(bytes(resp))}")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',     default='/dev/ttyUSB0')
    ap.add_argument('--baudrate', default=None, type=int,
                    help='Baudrate à tester (défaut: teste 115200, 57600, 1000000, 9600)')
    ap.add_argument('--ids',      default='1,2', help='IDs servo à tester')
    ap.add_argument('--timeout',  default=0.5,  type=float)
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(',')]

    baudrates = [args.baudrate] if args.baudrate else [115200, 57600, 1000000, 9600, 38400]

    print('=' * 65)
    print(f'  RS485 DIAGNOSTIC v2 — {args.port}')
    print('=' * 65)

    # Vérifier que le port existe
    import os
    if not os.path.exists(args.port):
        print(f'\n❌ Port {args.port} introuvable!')
        print('  Ports disponibles:')
        for p in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
            if os.path.exists(p):
                print(f'    ✓ {p}')
        sys.exit(1)

    # Vérifier les permissions
    if not os.access(args.port, os.R_OK | os.W_OK):
        print(f'\n❌ Permissions insuffisantes sur {args.port}')
        print(f'  → sudo chmod 666 {args.port}')
        print(f'  → ou: sudo usermod -a -G dialout $USER')
        sys.exit(1)

    found_something = False

    for baud in baudrates:
        print(f'\n{"─"*50}')
        print(f'  Baudrate: {baud}')
        print(f'{"─"*50}')

        try:
            ser = serial.Serial(
                port=args.port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=args.timeout
            )
        except serial.SerialException as e:
            print(f'  ❌ Impossible d\'ouvrir le port: {e}')
            continue

        time.sleep(0.1)  # laisser le port se stabiliser

        for sid in ids:
            print(f'\n  [PING → S{sid}]')
            ok = test_ping(ser, sid, timeout=args.timeout)
            if ok:
                found_something = True
                print(f'\n  [READ position → S{sid}]')
                test_read(ser, sid, addr=132, length=4, timeout=args.timeout)
            time.sleep(0.1)

        # Test broadcast (ID=0xFE)
        print(f'\n  [PING → BROADCAST (0xFE)]')
        pkt = ping_packet(0xFE)
        ser.reset_input_buffer()
        time.sleep(0.05)
        ser.reset_input_buffer()
        print(f'  TX: {hex_str(pkt)}')
        ser.write(pkt)
        ser.flush()
        time.sleep(0.3)
        resp = ser.read(256)
        if resp:
            print(f'  RX ({len(resp)} octets): {hex_str(resp)}')
            found_something = True
        else:
            print(f'  RX: silence')

        ser.close()

        if found_something:
            print(f'\n✓ Communication détectée à {baud} baud!')
            break

    print('\n' + '=' * 65)
    if not found_something:
        print('RÉSULTAT: SILENCE TOTAL sur tous les baudrates et IDs')
        print()
        print('CAUSES POSSIBLES (dans l\'ordre de probabilité):')
        print()
        print('  1. ALIMENTATION ESP32 COUPÉE')
        print('     → Vérifier que les ESP32 sont alimentés (LED allumée?)')
        print('     → Vérifier la tension sur VIN/3.3V des ESP32')
        print()
        print('  2. CÂBLAGE RS485 DÉCONNECTÉ')
        print('     → Vérifier les connexions A/B entre le module RS485')
        print('        et les ESP32')
        print('     → Vérifier GND commun entre Jetson et ESP32')
        print()
        print('  3. MODULE RS485 DÉFECTUEUX / DE BLOQUÉ')
        print('     → DE/RE de l\'adaptateur USB-RS485 toujours en TX')
        print('     → Tester avec un autre adaptateur')
        print()
        print('  4. FIRMWARE ESP32 PLANTÉ')
        print('     → Brancher USB sur l\'ESP32, ouvrir Serial Monitor @115200')
        print('     → Le firmware v1.10 affiche "[DXL] Prêt" au démarrage')
        print('     → Si rien → flasher le firmware')
        print()
        print('  5. BAUDRATE DIFFÉRENT')
        print('     → Le firmware utilise peut-être 57600 ou 1Mbps')
        print('     → Vérifier #define RS485_BAUDRATE dans le firmware')
    else:
        print('✓ Communication RS485 fonctionnelle!')
        print()
        print('Prochaines étapes:')
        print('  ros2 run dynamixel_motors dynamixel_node')
    print('=' * 65)


if __name__ == '__main__':
    main()

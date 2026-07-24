#!/usr/bin/env python3
"""
stm32_bench.py -- talk to the STM32 with NO ROS INVOLVED.

THIS IS THE MOST IMPORTANT DEBUGGING TOOL IN THE PROJECT.

If this does not work, nothing above it will, and you will waste a day blaming
Nav2 for what is actually a swapped TX/RX pair. Robot on blocks, wheels in the
air, run this FIRST.

    pip3 install pyserial

    # Is it alive? Just watch the feedback frames.
    python3 stm32_bench.py --watch

    # Drive both wheels forward for 3 s
    python3 stm32_bench.py --left 500 --right 500 --secs 3

    # Spin in place
    python3 stm32_bench.py --left 500 --right -500 --secs 3

    # Microstep verification (see below)
    python3 stm32_bench.py --verify-steps

WHAT TO CHECK, IN THIS ORDER
  1. --watch prints "$F,<l>,<r>,<lsps>,<rsps>,<flags>,<seq>*XX" at ~50 Hz.
     If not: baud, TX/RX orientation, common ground, or the crystal (see
     stm32/README.md -- an 8 vs 25 MHz mixup breaks baud AND step rate together).

  2. --left 500 --right 500: BOTH wheels turn so the robot would go FORWARD.
       Robot would SPIN      -> flip exactly ONE of MB_*_DIR_INVERT
       Robot would go BACKWARD -> flip BOTH

  3. Step counts climb POSITIVE when driving forward.

  4. --verify-steps: commands exactly one wheel revolution and asks you to check.
     If the wheel turns TWICE, your DRV8825 microstep jumpers say 1/8, not 1/16,
     and every distance the robot ever reports will be off by 2x.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pip3 install pyserial')


def crc8(data: bytes) -> int:
    """CRC-8/ATM, poly 0x07, init 0x00. Identical to MB_Crc8 in the firmware."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(payload: str) -> bytes:
    p = payload.encode()
    return b'$' + p + b'*' + f'{crc8(p):02X}'.encode() + b'\n'


FLAGS = {
    0x01: 'ENABLED', 0x02: 'ESTOP', 0x04: 'CMD_TIMEOUT',
    0x08: 'CRC_ERR', 0x10: 'OVERRUN', 0x20: 'CLAMPED',
}


def decode(line: str):
    if not line.startswith('$') or '*' not in line:
        return None
    payload, _, crc_hex = line[1:].partition('*')
    try:
        if crc8(payload.encode()) != int(crc_hex[:2], 16):
            return 'BAD CRC'
    except ValueError:
        return None
    f = payload.split(',')
    if f[0] != 'F' or len(f) != 7:
        return None
    ls, rs, lv, rv, fl, sq = (int(x) for x in f[1:])
    names = [n for bit, n in FLAGS.items() if fl & bit] or ['-']
    return (f'steps L={ls:<8} R={rs:<8} | sps L={lv:<6} R={rv:<6} | '
            f'seq={sq:<5} | {",".join(names)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyAMA0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--left', type=int, default=0, help='steps/s')
    ap.add_argument('--right', type=int, default=0, help='steps/s')
    ap.add_argument('--secs', type=float, default=2.0)
    ap.add_argument('--watch', action='store_true')
    ap.add_argument('--verify-steps', action='store_true')
    ap.add_argument('--steps-per-rev', type=int, default=3200)
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    def send(p):
        ser.write(frame(p))
        ser.flush()

    def rx():
        line = ser.readline().decode(errors='ignore').strip()
        return decode(line) if line else None

    send('P')
    time.sleep(0.2)

    if a.watch:
        print(f'Watching {a.port}. Ctrl-C to stop.\n')
        try:
            while True:
                d = rx()
                if d:
                    print(d)
        except KeyboardInterrupt:
            pass
        ser.close()
        return

    if a.verify_steps:
        print('MICROSTEP VERIFICATION')
        print(f'  Commanding exactly {a.steps_per_rev} steps '
              f'(= ONE wheel revolution at 1/16 microstepping).')
        print('  Put a mark on the wheel. Watch it.\n')
        input('  Press Enter to run...')
        send('R')
        send('E,1')
        time.sleep(0.2)
        rate = 400
        dur = a.steps_per_rev / rate
        t0 = time.time()
        while time.time() - t0 < dur:
            send(f'V,{rate},{rate}')
            time.sleep(0.05)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            send('V,0,0')
            time.sleep(0.05)
        time.sleep(0.3)
        last = None
        for _ in range(20):
            d = rx()
            if d:
                last = d
        print(f'\n  final: {last}')
        print('\n  Did the wheel turn EXACTLY ONE revolution?')
        print('    YES -> steps_per_rev is correct.')
        print('    TWO revolutions -> your M2/M1/M0 jumpers say 1/8, not 1/16.')
        print('       Fix the jumpers (M2=HIGH, M1=LOW, M0=LOW) or set')
        print('       geometry.microstepping: 8 in robot.yaml and regenerate.')
        send('E,0')
        ser.close()
        return

    print(f'Reset, enable, then V,{a.left},{a.right} for {a.secs}s')
    send('R')
    send('E,1')
    time.sleep(0.2)

    # The STM32 stops after a 500 ms command timeout, so we MUST keep re-sending.
    # That is not a bug -- it is the watchdog that stops a runaway robot when ROS
    # dies.
    t0, last_print = time.time(), 0.0
    while time.time() - t0 < a.secs:
        send(f'V,{a.left},{a.right}')
        time.sleep(0.05)
        d = rx()
        if d and time.time() - last_print > 0.25:
            print(f'[{time.time()-t0:5.2f}s] {d}')
            last_print = time.time()

    print('ramping down...')
    t0 = time.time()
    while time.time() - t0 < 1.5:
        send('V,0,0')
        time.sleep(0.05)

    time.sleep(0.3)
    d = rx()
    if d:
        print(f'[final] {d}')

    send('E,0')
    print('drivers disabled. done.')
    ser.close()


if __name__ == '__main__':
    main()

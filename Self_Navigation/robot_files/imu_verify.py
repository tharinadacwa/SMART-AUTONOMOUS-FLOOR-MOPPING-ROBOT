#!/usr/bin/env python3
import smbus2, time

MPU = 0x68
bus = smbus2.SMBus(1)

# Wake MPU9250
bus.write_byte_data(MPU, 0x6B, 0x00)
time.sleep(0.1)
# Enable I2C bypass so we can reach the AK8963 magnetometer directly
bus.write_byte_data(MPU, 0x37, 0x02)   # INT_PIN_CFG: BYPASS_EN
time.sleep(0.1)

AK = 0x0C  # AK8963 magnetometer address
def mag_present():
    try:
        who = bus.read_byte_data(AK, 0x00)  # WIA register, should be 0x48
        return who == 0x48, who
    except Exception as e:
        return False, None

def rd(addr, reg):
    h = bus.read_byte_data(addr, reg); l = bus.read_byte_data(addr, reg+1)
    v = (h << 8) | l
    return v - 65536 if v > 32767 else v

ok, who = mag_present()
print(f"[MAG] AK8963 WHO_AM_I = {hex(who) if who is not None else 'NO RESPONSE'} "
      f"({'REAL magnetometer present' if ok else 'MISSING / FAKE CHIP'})")

# Configure magnetometer to continuous 16-bit mode if present
if ok:
    bus.write_byte_data(AK, 0x0A, 0x16)
    time.sleep(0.1)

print("\nMove/rotate the board by hand and watch the numbers change:\n")
for _ in range(40):
    ax, ay, az = rd(MPU,0x3B), rd(MPU,0x3D), rd(MPU,0x3F)
    gx, gy, gz = rd(MPU,0x43), rd(MPU,0x45), rd(MPU,0x47)
    line = f"ACC {ax:6d} {ay:6d} {az:6d}   GYR {gx:6d} {gy:6d} {gz:6d}"
    if ok:
        # AK8963 is little-endian, data 0x03..0x08, must read ST2(0x09) to latch
        mx = rd(AK,0x03); my = rd(AK,0x05); mz = rd(AK,0x07); bus.read_byte_data(AK,0x09)
        line += f"   MAG {mx:6d} {my:6d} {mz:6d}"
    print(line)
    time.sleep(0.2)

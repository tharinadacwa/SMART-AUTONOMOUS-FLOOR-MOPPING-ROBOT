# L298N DC-motor wiring — minibot (STM32F407VET6)

4 brushed DC motors, 2× L298N, 12 V supply. Direction is fixed in firmware:
on **each** driver one motor runs **CW** and the other **CCW**.

Only 8 pins were free, and all 8 are used as L298N `IN` lines. There is **no
spare pin for the enable inputs**, so on each driver the tied-together
`ENA+ENB` node must sit at **+5 V** (both channels permanently enabled).
Direction / stop is done entirely with the `IN` pins.

---

## 1. Signal wiring — STM32 → L298N IN pins

### Driver 1  (Motors 1 & 2)
| STM32 pin | L298N IN | Motor / output | Fixed direction |
|-----------|----------|----------------|-----------------|
| PD0       | IN1      | Motor 1 (OUT1/OUT2) | CW  → IN1=H, IN2=L |
| PD1       | IN2      | Motor 1        |                    |
| PD2       | IN3      | Motor 2 (OUT3/OUT4) | CCW → IN3=L, IN4=H |
| PD3       | IN4      | Motor 2        |                    |

### Driver 2  (Motors 3 & 4)
| STM32 pin | L298N IN | Motor / output | Fixed direction |
|-----------|----------|----------------|-----------------|
| PA2       | IN1      | Motor 3 (OUT1/OUT2) | CW  → IN1=H, IN2=L |
| PA3       | IN2      | Motor 3        |                    |
| PE14      | IN3      | Motor 4 (OUT3/OUT4) | CCW → IN3=L, IN4=H |
| PE15      | IN4      | Motor 4        |                    |

The STM32 outputs 3.3 V logic. The L298N inputs are TTL and read 3.3 V as a
solid HIGH (input-high threshold ≈ 2.3 V), so **no level shifter is needed**.

---

## 2. Enable pins (ENA + ENB)  — the important part

On each L298N you have tied `ENA` and `ENB` together. That combined node must
be held at **+5 V** so both channels are always enabled. Two equivalent ways:

- **Easiest:** keep **both** on-board `ENA` and `ENB` jumper caps installed.
  Each cap ties its enable to the board's internal +5 V — done.
- **If you removed the caps** and soldered ENA↔ENB together: run **one wire**
  from that combined node to the board's **+5 V** pin.

There is no PWM speed control in this build (enable is fixed HIGH). The motors
run at whatever speed the 12 V rail gives. See §5 if you want speed control.

---

## 3. Power wiring  (do this carefully — grounding is the #1 mistake)

For **each** L298N:

| L298N terminal | Connect to |
|----------------|-----------|
| `+12V` (motor supply) | 12 V battery **+** |
| `GND`          | 12 V battery **−**  **and**  STM32 GND (shared) |
| `+5V`          | leave the 5V-EN jumper **ON** → this pin is the board's own regulated 5 V, used internally for the ENA/ENB caps. Do **not** wire it to any other 5 V source. |

**Common ground is mandatory.** The STM32 GND, both L298N GND terminals, and
the 12 V battery minus must all be joined. Without a shared ground the IN
signals have no reference and the motors behave randomly or not at all.

Do **not** power the STM32 board from the L298N +5 V unless that is genuinely
your only supply and you understand the current budget — keep the STM32 on its
own USB / 5 V supply and just share ground.

```
                 12 V battery
                  +        -
                  |        |
        +---------+        +----------------+-----------------+
        |                                   |                 |
   [L298N #1 +12V]                     [L298N #1 GND]   [L298N #2 GND]---+
   [L298N #2 +12V]                          |                            |
                                            +-------- STM32 GND ---------+
                                                     (common ground)
```

---

## 4. Motor output wiring & direction

- Driver 1: **OUT1/OUT2 → Motor 1**, **OUT3/OUT4 → Motor 2**
- Driver 2: **OUT1/OUT2 → Motor 3**, **OUT3/OUT4 → Motor 4**

"CW" / "CCW" are only labels — real spin direction depends on which way you
land the two motor leads on `OUTx`. **If a motor spins the wrong way**, either
swap its two output wires, or swap `DCM_CW`/`DCM_CCW` for that motor in
`Core/Src/minibot_dcmotor.c` (function `DCM_Run`).

---

## 5. Firmware hooks (already in the project)

- `Core/Inc/minibot_dcmotor.h`, `Core/Src/minibot_dcmotor.c` — the module.
- `main()` calls `DCM_Init()` (pins → outputs, all stopped) then `DCM_Run()`
  (applies M1 CW, M2 CCW, M3 CW, M4 CCW). Comment out `DCM_Run()` to leave the
  motors stopped at boot.
- `DCM_Set(DCM_MOTOR_2, DCM_STOP)` etc. lets you drive any motor individually.
- The stepper code, UART protocol, TIM7 and all original pins are untouched.

### If you later want speed control
You'd need to free a pin per driver for a PWM enable line: stop tying ENA/ENB
to 5 V, feed the combined ENA/ENB from a timer PWM output instead, and keep the
IN pins for direction. That needs 2 more pins than are currently free, so it
would mean giving up one of the existing functions or moving to a board with
more free pins.

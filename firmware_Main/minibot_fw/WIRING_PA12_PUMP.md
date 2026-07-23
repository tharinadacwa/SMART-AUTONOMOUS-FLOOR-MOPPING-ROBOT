# PA12 pump switch — low-side IRLZ44N (STM32F407VET6)

Firmware drives PA12: **HIGH 3 s (pump ON), LOW 5 s (pump OFF)**, forever.
PA12 → MOSFET gate → pump switches on and off. This is a low-side switch
(load on the high side, MOSFET between load and ground).

## Correct connections

```
        +12V ─────────────┬──────────────► pump (+)
                          │
                     (cathode)
                       1N4007          ◄── flyback diode ACROSS the pump
                      (anode)
                          │
        pump (−) ─────────┴───────────┬──► IRLZ44N DRAIN
                                      │
   PA12 ──[220 Ω]──┬── IRLZ44N GATE   │
                   │                  │
                [10 kΩ]           IRLZ44N SOURCE ──► GND
                   │
                  GND

   Common ground: STM32 GND = MOSFET source = 12V supply (−)
```

- **PA12 → 220 Ω → gate:** good. The 220 Ω limits the gate-charge inrush.
- **Gate → 10 kΩ → GND (add this):** a pull-down that holds the MOSFET OFF
  while PA12 is high-impedance (during reset/boot/flashing). Without it the
  pump can twitch on at power-up before firmware runs. Strongly recommended.
- **Drain = pump (−), Source = GND:** correct for a low-side switch.
- **Common ground is mandatory:** STM32 GND, MOSFET source, and the 12 V
  supply minus must all be the same node, or the 3.3 V gate signal has no
  reference and the MOSFET won't switch cleanly.

## The flyback diode — check its placement

It must sit **across the pump**, cathode (stripe) to **+12 V**, anode to the
**pump(−)/drain** node. That gives the motor's coil current a path to
free-wheel when the MOSFET turns off (pump− → diode → +12 V), clamping the
drain spike to ~12.7 V.

A diode from the drain to **ground** (a common miswire) does **not** clamp the
turn-off spike, because it doesn't complete the coil's current loop back to the
+12 V rail — the drain can still fly well above 12 V and stress the MOSFET. If
that's how it's wired now, move the diode across the pump as above.

## IRLZ44N at a 3.3 V gate

The IRLZ44N is a logic-level MOSFET, so 3.3 V does turn it on. But its on-
resistance is specified at Vgs = 5 V / 4 V; at 3.3 V R_DS(on) is a bit higher,
so it runs a little warmer under load. For a small pump this is fine. For a
higher-current pump, either drive the gate at 5 V (e.g. a small
gate-driver/level-shift transistor) or add a heatsink, and keep the gate 220 Ω
where it is.

## Changing the timing

Edit `Core/Src/minibot_signal.c`:

    #define SIG_HIGH_MS   3000u   /* pump ON  time (ms) */
    #define SIG_LOW_MS    5000u   /* pump OFF time (ms) */

The pump starts in the OFF phase at boot (safe). To start with the ON phase
instead, set `s_level = 1` in `Sig_Init()` and drive the pin `GPIO_PIN_SET`
there.

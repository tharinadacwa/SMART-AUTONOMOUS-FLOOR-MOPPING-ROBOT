<div align="center">

# Charging Sub-System

### LiPo charging hardware and firmware for the Smart Autonomous Floor Mopping Robot

A USB-C Power Delivery charger that charges and balances the robot's onboard
LiPo pack — built around an STM32G0 MCU and a TI BQ25703A buck-boost charger.

</div>

<br>

<div align="center">
  <img src="EDR_LIPO/EDE_LIPO_photo.jpeg" alt="Assembled and tested EDR_LIPO charging board" width="600">
  <br>
  <em>Assembled, populated, and bench-tested charging board</em>
</div>

---

## Overview

This sub-system takes power from a USB Type-C PD source, negotiates a supply
voltage, and charges a multi-cell lithium-polymer pack through a TI BQ25703A
programmable buck-boost charge controller supervised by an STM32G0. Per-cell
voltages are read through the MCU's ADC, and passive balancing keeps the pack
matched during charge. The design is adapted from the open-source LiPow charger.

It is split into two parts — the **hardware** (Altium PCB) and the **firmware**
(STM32G0) — each documented in its own folder.

## Contents

| Folder | Description |
|---|---|
| **[`EDR_LIPO/`](EDR_LIPO)** | Hardware — Altium schematics, PCB layout, renders, and board photos for the LiPo charging board |
| **[`edr_Lipo_1v1/`](edr_Lipo_1v1)** | Firmware (v1.1) — STM32G0 project: USB-PD negotiation, BQ25703A charge control, and cell balancing |

## Key specifications

| Parameter | Value |
|---|---|
| Function | USB-C PD LiPo charger and balancer |
| MCU | STM32G071CBTx (Arm Cortex-M0+, FreeRTOS) |
| Input | USB Type-C, USB Power Delivery *(requests up to 3 A)* |
| Charge controller | TI BQ25703A buck-boost, over I²C |
| Charge output / balance | XT60 connector / JST-XH balance connector |
| Supported packs | 1S – 3S *(per firmware cell-count bitmasks)* |
| Status indicator | RGB LED 🌈 — charge / balance state |
| Board status | Assembled and bench-tested |

> Values in *italics* come from the firmware or the LiPow reference design. Confirm
> the current limit and cell count against your build before connecting a real pack.
> Full specs and protection thresholds are in each subfolder's README.

---

## Credits & License

This sub-system is a derivative of the open-source **LiPow** USB-C PD LiPo charger:

<https://hackaday.io/project/161771-usb-power-delivery-lipo-battery-charger>

Original hardware and firmware © the LiPow project authors, licensed under the
**GNU General Public License v3.0**. Because this work derives from it, it is
**also covered by the GPLv3** — keep the attribution and a copy of the `LICENSE`
with the source.

<div align="center">
<br>
<sub>Board revision, firmware port, integration and testing for the Smart Autonomous Floor Mopping Robot by <b>@tharinadacwa</b></sub>
</div>
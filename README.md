# TCU-349
### Transponder Control Unit — Adafruit ItsyBitsy M4 Express

---

## Overview

Firmware for a ground-based Transponder Control Unit that generates precisely timed SPI frames and strobe pulses to drive a transponder receiver. Timing is slot-based at 64µs per slot, with SPI data sent at each slot midpoint and a strobe latch fired at each slot boundary.

## How It Works

The firmware runs a continuous repeating cycle of **sequence steps** and **gap steps**.

**Sequence steps** transmit either an Elevation (EL) or Azimuth (AZ) scan function. Each function begins with a 25-slot preamble followed by antenna/data content, with special strobes fired at exact microsecond boundaries for pause, fro-scan, and end events.

**Gap steps** transmit datawords (BDW and ADWA) and heartbeat frames between sequences.

A unified event system handles all fixed-time events — function starts, special strobes, and dataword transitions — by suppressing the regular slot within 64µs before each event and firing SPI and strobe at precise absolute times.

## Modes

| Mode | Description |
|------|-------------|
| `MODE_AZ` | Azimuth scan — single function starting at 10ms into each sequence |
| `MODE_EL` | Elevation scan — three sub-functions (EL0, EL1, EL2) per sequence |
| `MODE_BAZ` | Back-azimuth scan — similar structure to AZ with different timing and antenna sequencing *(incomplete)* |

## Hardware

| Pin | Function |
|-----|----------|
| 10 | SPI Chip Select |
| 9 | Strobe output |
| 11 | Sync input |
| 12 | Sync output |

> SPI runs at 1MHz, MSBFIRST, MODE0. Pin assignments marked `&&` in source require final hardware confirmation.

## Files

| File | Description |
|------|-------------|
| `tcu.cpp` | All firmware logic — frame builders, event system, setup/loop |
| `tcu_header.h` | Defines, structs, externs, and function prototypes |

---

## Contributors

- 
- 
-
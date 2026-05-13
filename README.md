# TCU-349
### Transponder Control Unit — Adafruit ItsyBitsy M4 Express

---

## Overview

Firmware for a ground-based Transponder Control Unit that generates precisely timed SPI frames and strobe pulses to drive a transponder receiver. Timing is slot-based at 64µs per slot, with SPI data sent at each slot midpoint and a strobe latch fired at each slot boundary.

## How It Works

The firmware runs a continuous repeating cycle of sequence steps and gap steps.

**Sequence steps** transmit either an Elevation (EL), Azimuth (AZ) or Back Azimuth (BAZ) scan function. Each function begins with a 25-slot preamble followed by antenna/data content, with special strobes fired at exact microsecond boundaries for pause, fro-scan, and end events. Back Azimuth and Azimuth support Datawords including Basic and Auxillary Datawords being sent during pause times VIA DPSK

**Gap steps** heartbeat frames between sequences.

A unified event system handles all fixed-time events — function starts, special strobes, and dataword transitions — by suppressing the regular slot within 64µs before each event and firing SPI and strobe at precise absolute times.

## Modes

| Mode | Description |
|------|-------------|
| `MODE_AZ` | Azimuth scan — single function starting at 10ms into each sequence |
| `MODE_EL` | Elevation scan — three sub-functions (EL0, EL1, EL2) per sequence |
| `MODE_BAZ` | Back-azimuth scan — similar structure to AZ with different timing and antenna sequencing |

## Byte Format

| Bit (MSB to LSB) | Function |
|-----|----------|
| 8 | Transmitter Enable |
| 7 | DPSK |
| 6 | TO/FRO (A high value represents a switch from one to the other) |
| 5 | Scanning beam Enable |
| 4 | OCI MSB |
| 3 | OCI |
| 2 | OCI LSB |
| 1 | K constant selector bit (0 = (k == 1), 1 = (k != 1)) |


## Hardware

| Pin | Function |
|-----|----------|
| 10 | SPI Chip Select |
| 9 | Strobe output |
| 11 | Sync input |
| 12 | Sync output |

> SPI runs at 1MHz, MSBFIRST, MODE0. Modifiable Values are marked with && within the final code

## Files

| File | Description |
|------|-------------|
| `tcu.cpp` | All firmware logic — frame builders, event system, setup/loop |
| `tcu_header.h` | Defines, structs, externs, and function prototypes |

---

## Contributors

- Theo Poulos
- Ian Cox
- Danielle Burton
- Siena Evers
# Zigbee Folding-Shutter Controller

An open-hardware Zigbee controller for 24 V DC folding-shutter window drives, built around the ESP32-C6.
It fully replaces the proprietary control unit of a motorized folding-shutter system and fits into a deep
flush-mount wall box, where it drives the two gear motors of one window directly.

## Features

- Drives **two 24 V DC gear motors** per window with covering-wing sequencing
  (the covering wing opens first and closes last)
- **Current-sensing end-stop detection** — no limit switches; travel limits and current
  thresholds are calibrated by a learn cycle
- **Obstacle detection** with automatic reversal
- Soft start / soft stop via PWM
- **8 monitored dry-contact inputs**: 4 window contacts + 4 tamper/continuity loops
- Local **wall-button input** (open/close) in addition to radio control
- **Zigbee 3.0** — integrates with Zigbee2MQTT and Home Assistant
- Two-board stack sized for a deep flush-mount box:
  - *power board*: input protection, motor drivers with current feedback, 24 V → 3.3 V buck
  - *logic board*: ESP32-C6 module, sensor input conditioning, buttons, status LED

## Project status

⚠️ **Work in progress.** This project is under active development and has not yet been
field-proven.

| Part | Status | Published |
|------|--------|-----------|
| Firmware | in development | [`firmware/`](firmware/) |
| PCB design files (KiCad) | designed, awaiting build & field test | not yet — see below |
| Enclosure (STEP/STL) | in design | not yet |

The PCB design files will be published **after** the first batch has been built and tested in
real windows — please don't manufacture boards from an unproven design. The printable
enclosure files follow once they have been verified on real hardware.

## Repository layout

```
firmware/    ESP32-C6 Zigbee firmware (ESP-IDF) + flashing / integration guide
hardware/    KiCad schematics & PCB layouts (published after field testing)
enclosure/   3D-printable enclosure, STEP + STL (published after verification)
```

## Safety

This device switches motor loads and moves heavy shutter wings. It is a hobby project,
provided **without any warranty** — see the disclaimer below. Anything you build or install
based on this repository is at your own risk; make sure moving shutters can never endanger
people or pets, and always disconnect the 24 V supply before working on the wiring.

## Disclaimer

THE DESIGN FILES, FIRMWARE AND DOCUMENTATION IN THIS REPOSITORY ARE PROVIDED "AS IS",
WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED. IN NO EVENT SHALL THE AUTHORS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY ARISING FROM THE USE OF THIS PROJECT.

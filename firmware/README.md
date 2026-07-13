# Firmware

Zigbee firmware for the ShutterNode controller (ESP32-C6), built on ESP-IDF and the
Espressif Zigbee SDK.

⚠️ **Status: untested on real hardware.** The firmware compiles and implements the full
feature set described below, but the first boards have not been assembled yet. Expect
changes once hardware bring-up starts.

## What it does

- **Zigbee 3.0 router** exposing a standard *window covering* device — works with
  Zigbee2MQTT (external converter provided) and through it with Home Assistant
- Drives the **two wing motors** of one folding shutter with the covering-wing sequence
  (wing 1 opens first, closes last; 6 s stagger) and PWM soft start/stop
- **End stops by motor current** — no limit switches; a *learn cycle* measures travel
  time and normal running current per motor and derives the stop thresholds
- **Obstacle detection**: a current spike mid-travel stops the pair and reverses it back
  to where the move started; a spike right after starting (frozen shutter) is treated as
  "already at the end"
- Position control (`go to x %`) via dead-reckoning on the learned travel time
- **4 window contacts + 4 tamper loops** exposed as individual sensors,
  **2 wall buttons** (open/close, press while moving = stop) working fully offline
- Supply-rail voltage measurement, reported over Zigbee
- Setup button: **short press = learn cycle**, **hold 5 s = leave network / re-pair**
  (factory reset of the Zigbee part; the motor calibration is kept)

## Building

1. Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/get-started/index.html)
   **v5.3 or newer** (v5.5 tested) including the esp32c6 toolchain.
2. In an ESP-IDF environment shell:

   ```sh
   cd firmware
   idf.py set-target esp32c6
   idf.py build
   ```

   The Zigbee libraries are pulled in automatically by the IDF component manager on the
   first build.

## Flashing

The logic board is programmed over the 6-pin UART header (3V3, GND, EN, TX, RX, BOOT)
using any 3.3 V USB-UART adapter:

1. Connect 3V3, GND, and cross TX/RX (adapter TX → header RX and vice versa).
2. Enter the bootloader: hold BOOT low (button or header pin), pulse EN low, release BOOT.
3. Flash and watch the log:

   ```sh
   idf.py -p <PORT> flash monitor
   ```

Subsequent flashes work the same way; the board has no USB connector.

## Commissioning & first use

1. **Pairing:** a factory-new device starts searching for a network on first boot
   (status LED blinks fast). Enable *permit join* in Zigbee2MQTT and it will appear.
   To re-pair later, hold the setup button for 5 s.
2. **Zigbee2MQTT:** copy [`zigbee2mqtt/shutternode.mjs`](zigbee2mqtt/shutternode.mjs)
   into the Zigbee2MQTT `data/external_converters/` directory and restart Zigbee2MQTT
   *before* pairing.
3. **Learn cycle:** after installation, press the setup button briefly (LED blinks
   slowly). The shutter closes fully, opens fully and closes again while the firmware
   measures travel times and motor currents. Until a learn cycle has run, conservative
   defaults are used and positioning is approximate.
4. If a wing moves in the wrong direction, swap the two wires of that motor at its
   screw terminal.

## Layout

```
main/board.h     pin map & hardware constants (current-sense scaling, divider ratios)
main/shutter.c   motor state machine: sequencing, ramps, current end stops, learn cycle
main/inputs.c    shift-register scan (contacts, tamper loops, wall buttons), setup button
main/sense.c     ADC: motor current sense, supply rail
main/zb_app.c    Zigbee endpoints, clusters, commissioning
main/main.c      wiring it all together, status LED
zigbee2mqtt/     external converter for Zigbee2MQTT
```

## Notes

- The device joins as a **router** (it is mains powered) and strengthens the mesh.
- Motor current thresholds derived by the learn cycle include the part spread of the
  driver's current mirror (±15 %); re-run the learn cycle after mechanical changes.
- OTA updates are not implemented yet.

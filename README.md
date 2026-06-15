# FPV Head Tracker — ESP32-C3 + BNO085 → ELRS Backpack

A DIY wireless FPV head tracker. A **BNO085 IMU** mounted on the goggles sends
head orientation over **ESP-NOW** to a **RadioMaster Nomad TX backpack**, which
feeds pan/tilt as trainer input into **EdgeTX**, then out over **ELRS** to a quad
running **Betaflight** (servo channel forwarding to a pan/tilt gimbal).

Based on [druckgott's ELRS-Backpack-Example-ESPNOW](https://github.com/druckgott/ELRS-Backpack-Example-ESPNOW)
(`fix_esp32_work` branch). The ESP-NOW / fake-VTX core is **his work** — this repo
only adds the BNO085 head-tracking layer on top.


## Hardware

- **ESP32-C3 SuperMini** (external-antenna version recommended — the PCB-antenna
  ones drop ESP-NOW packets)
- **BNO085 IMU** (GY-BNO08X board), I2C address `0x4B`
- **RadioMaster Nomad** TX backpack (firmware 1.5.5)
- Push button on **GPIO3 → GND** (recenter)


## Radio side (EdgeTX / backpack)

- Backpack **HT Start Channel → EdgeTX** (not an AUX channel). On an AUX value it
  directly overrides receiver channels instead of feeding the trainer.
- Trainer **Mode → Master/CRSF**
- Mixes: `CHx ← TR1` (pan), `CHy ← TR3` (tilt)
- Keep head tracking on **lower channels** (e.g. CH11/CH12). High channels
  (CH15/16) were unreliable depending on switch mode / packet rate.


## Head-tracking features (this repo)

- **Quaternion-based** pan/tilt with **decoupled axes** (avoids Euler-angle
  cross-coupling between pan and tilt)
- **Recenter button** on GPIO3 — captures the current orientation as the new center
- Travel **limited to ±100°** per axis

## Configuration

Set your own UID in `main.cpp`:

```cpp
uint8_t UID[6] = {0,0,0,0,0,0};   // get it from the ELRS web-flasher using your bind phrase
```

The placeholder `{0,0,0,0,0,0}` will **not** bind — you must enter your own UID.

## Build

PlatformIO, environment `esp32c3`:

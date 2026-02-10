# M5StickC Plus2 Low-Power Door Sensor

Production-oriented firmware and deployment assets for an **M5StickC Plus2** door sensor with:

- Reed-switch door state detection (`GPIO26`)
- IMU-based tamper detection while door is closed
- MQTT event publishing for PC integrations (Node-RED/Mosquitto)
- Optional Telegram alerting (direct from device)
- Deep-sleep-first lifecycle for battery operation

## Repository Contents

- `firmware/door_sensor.ino` — Main firmware (modularized by functional sections)
- `firmware/secrets.example.h` — Copy-to-config template for Wi-Fi/MQTT/Telegram
- `nodered/door_sensor_flow.json` — Importable Node-RED flow for MQTT -> Telegram
- `docs/android-app-selection.md` — Android application options and recommendations
- `docs/test-validation-checklist.md` — Validation checklist and test procedure

---

## 1) Hardware Setup

### Mandatory hardware

- M5StickC Plus2
- Magnetic reed switch
- Magnet

### Wiring

- Reed switch one side -> **GPIO 26**
- Reed switch other side -> **GND**
- Firmware uses `INPUT_PULLUP` (active-low switch)

> If your installation yields inverted semantics (OPEN/CLOSED reversed), set `DOOR_CLOSED_LEVEL` in firmware.

---

## 2) Firmware Build/Flash (ESP32 Arduino Core)

1. Install Arduino IDE 2.x
2. Install **ESP32 by Espressif Systems** board package (Arduino core)
3. Install libraries:
   - `M5Unified`
   - `PubSubClient`
   - `ArduinoJson`
4. Copy `firmware/secrets.example.h` to `firmware/secrets.h`
5. Fill in Wi-Fi + MQTT configuration in `secrets.h`
6. Open `firmware/door_sensor.ino`
7. Select matching board for M5StickC Plus2 and flash

---

## 3) Runtime Behavior (Boot -> Sleep)

1. Device wakes from ESP32 EXT0 (`GPIO26`) or timer
2. CPU is lowered to 80 MHz; LCD stays off
3. Door state is sampled/debounced
4. If state changed -> publish `OPEN`/`CLOSED`
5. If door closed -> IMU sampled for ~240 ms for tamper detection
6. If movement exceeds threshold and cooldown passed -> publish tamper
7. Optional heartbeat on timer wake
8. Wi-Fi is disconnected, then device returns to deep sleep

The firmware dynamically sets EXT0 wake level to the opposite of the current reed level, so both OPEN and CLOSED transitions can wake across sleep cycles.

No blocking loop is used; all work happens in `setup()` and `loop()` is empty.

---

## 4) MQTT Topic Contract

- `door/state` with event payload `OPEN` or `CLOSED`
- `door/tamper` with event payload `FORCE_DETECTED`
- `door/heartbeat` (optional)

Payload schema:

```json
{
  "device": "m5stickc-plus2",
  "event": "OPEN",
  "timestamp": "ISO8601-or-uptime-fallback",
  "battery_mv": 4012,
  "wake_reason": "EXT0"
}
```

---

## 5) Node-RED + Telegram (Recommended)

1. Install Mosquitto broker on PC
2. Install Node-RED
3. Import `nodered/door_sensor_flow.json`
4. Set broker host/credentials in MQTT config node
5. Set Telegram bot token + chat id in function/telegram nodes
6. Deploy flow

This flow subscribes to:

- `door/state`
- `door/tamper`
- `door/heartbeat`

and forwards human-readable alerts to Telegram.

---

## 6) Battery & Power Notes

- Deep sleep is the default mode
- Wi-Fi is enabled only during transmit window
- Expected active window per event is typically under ~2 seconds in normal networks
- Optional heartbeat interval can be changed to reduce background wakeups

For best battery life:

- Keep RSSI strong (reduces connect time)
- Avoid very frequent heartbeat
- Keep tamper threshold tuned to your mount vibration profile

---

## 7) Acceptance Validation

Use `docs/test-validation-checklist.md` for end-to-end verification:

- Door state reliability
- False positive tamper screening
- Deep sleep current / wake duty cycle
- End-to-end notification latency

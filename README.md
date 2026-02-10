# M5Stack Cplus2 Low-Power Door Sensor

Production-oriented firmware and deployment assets for an **M5Stack Cplus2** door sensor with:

- **Inbuilt sensor-based door state detection** (e.g., accelerometer, magnetometer)
- MQTT event publishing for PC integrations (Node-RED/Mosquitto)
- Optional Telegram alerting (direct from device)
- Deep-sleep-first lifecycle for optimal battery operation

## Repository Contents

- `firmware/main/door_sensor.cpp` — Main firmware (ESP-IDF)
- `nodered/door_sensor_flow.json` — Importable Node-RED flow for MQTT -> Telegram
- `docs/android-app-selection.md` — Android application options and recommendations
- `docs/test-validation-checklist.md` — Validation checklist and test procedure

---

## 1) Hardware Setup

### Mandatory hardware

- M5Stack Cplus2

### Wiring

- No external wiring required for door state detection, uses inbuilt sensors.

---

## 2) Firmware Build/Flash (ESP-IDF)

1. Ensure you have the ESP-IDF development environment set up.
2. Navigate to the `firmware/` directory.
3. Configure the project: `idf.py menuconfig` (set Wi-Fi, MQTT, and other configurations)
4. Build the project: `idf.py build`
5. Flash the firmware: `idf.py -p /dev/ttyUSB0 flash` (replace `/dev/ttyUSB0` with your serial port)

---

## 3) Runtime Behavior (Boot -> Sleep)

1. Device wakes from ESP32 EXT0 or timer
2. CPU is lowered to 80 MHz; LCD stays off
3. Inbuilt sensors are sampled/debounced to determine door state
4. If state changed -> publish `OPEN`/`CLOSED` via MQTT
5. Optional heartbeat on timer wake
6. Wi-Fi is disconnected, then device returns to deep sleep

The firmware dynamically sets EXT0 wake level (if applicable for inbuilt sensors) to minimize power consumption and ensure both OPEN and CLOSED transitions can wake across sleep cycles.

---

## 4) MQTT Topic Contract

- `door/state` with event payload `OPEN` or `CLOSED`
- `door/tamper` with event payload `FORCE_DETECTED` (if implemented with inbuilt IMU)
- `door/heartbeat` (optional)

Payload schema:

```json
{
  "device": "m5stickc-plus2",
  "event": "OPEN",
  "timestamp": "ISO8601-or-uptime-fallback",
  "battery_mv": 4012,
  "wake_reason": "EXT0" // Or other wake reason
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
- Tune sensor sampling and thresholds for optimal detection and power usage

---

## 7) Acceptance Validation

Use `docs/test-validation-checklist.md` for end-to-end verification:

- Door state reliability using inbuilt sensors
- Deep sleep current / wake duty cycle
- End-to-end notification latency

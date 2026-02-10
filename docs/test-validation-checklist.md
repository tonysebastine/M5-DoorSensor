# Test & Validation Checklist

## A. Functional Verification

- [ ] Door CLOSED event emitted exactly once on transition
- [ ] Door OPEN event emitted exactly once on transition
- [ ] No repeated duplicate events while state unchanged
- [ ] Tamper event only when door is CLOSED and movement exceeds threshold
- [ ] No tamper event during intentional OPEN/CLOSE motion

## B. Power Behavior

- [ ] Device enters deep sleep after handling event
- [ ] Wi-Fi disabled when idle (verify with AP logs/sniffer)
- [ ] Awake processing window <= 2 seconds under normal network
- [ ] Deep sleep current measured against target budget

## C. Wake Source Validation

- [ ] EXT0 wake (reed pin) works for real door transitions
- [ ] Timer wake (heartbeat) works at configured interval
- [ ] Wake reason reported correctly in payload

## D. Network/Notification Validation

- [ ] MQTT broker receives events on `door/state`
- [ ] MQTT broker receives events on `door/tamper`
- [ ] MQTT broker receives heartbeat on `door/heartbeat` (if enabled)
- [ ] Node-RED parses payload JSON correctly
- [ ] Telegram alert is delivered on each actionable event

## E. Reliability / False Alarm Screening

- [ ] 24-hour idle run without false tamper alarms
- [ ] Tamper cooldown prevents rapid repeated alerts
- [ ] Sensor mount vibration tested and threshold tuned

## F. Battery-Life Validation

- [ ] Record events/day and average awake time
- [ ] Estimate battery life from measured duty cycle
- [ ] Confirm >=12h in worst-case scenario
- [ ] Validate target multi-day operation in typical scenario


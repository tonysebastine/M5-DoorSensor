# Android App Selection Guide

## Recommended: Telegram

**Why**
- Reliable push delivery
- Low battery overhead
- Works well with Node-RED integration

**Use with this project**
- Device publishes MQTT events
- Node-RED formats alerts
- Telegram bot pushes to your phone in near real-time

## Alternative: Home Assistant Android App

**Why**
- Unified smart-home dashboard
- Great if already running Home Assistant

**Trade-off**
- More setup complexity than Telegram

## Alternative: MQTT Dash / MQTT Client Apps

**Why**
- Quick direct topic monitoring
- Good for commissioning and debugging

**Trade-off**
- Usually not true push notifications unless app is foreground/background-permitted

## Final recommendation

For fastest deployment and robust alerting, use:

1. Mosquitto on PC
2. Node-RED flow in this repo
3. Telegram app on Android


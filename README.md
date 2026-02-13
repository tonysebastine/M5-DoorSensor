# M5StickC Plus 2 - Smart Door Sensor

A high-performance codebase that transforms the **M5StickC Plus 2** into a smart, power-efficient Door Sensor.

## 🚀 Features
*   **Instant Alerts:** Detects door opening in **0.2 seconds**. Sends alerts via **MQTT** and **Telegram**.
*   **Smart Static IP:** Captures and saves your IP info, BSSID, and Channel on the first connection. Subsequent wake-ups are **<1 second** with Fast-Connect BSSID Pinning.
*   **10+ Hour Battery:** Uses smart `Light Sleep` to last ~20-30 hours on a single charge.
*   **Modern UI:**
    *   **Green Box:** Door Closed.
    *   **Red Box:** Door Open.
    *   **Grey Box:** Disarmed (Silent).
    *   **WiFi Status:** Shows `OK`, `FAIL`, or `ADDR...` with the **WiFi SSID** displayed at the bottom.
*   **WiFi Configuration Portal:** Configure WiFi credentials without recompiling.
*   **Auto-Orientation:** The display automatically rotates based on mounting position.
*   **Arm/Disarm:** Toggle alerts directly from the device.

---

## 🎮 Controls & Usage

### 1. **Wake Screen / Check Status**
*   **Press Button A (Front):** Turns the screen **ON**.
*   **Press Button A again:** Turns the screen **OFF**.
*   (Auto-off after 30 seconds).

### 2. **Arm / Disarm (Silent Mode)**
*   Wake the screen.
*   **Hold Button A (Front) for 1 second.**
*   The screen will flash **White**.
*   **Grey Box** = Disarmed (No alerts will be sent).
*   **Colored Box** = Armed (Alerts active).

### 3. **Calibration (One-Time Setup)**
*   Mount the device on your door using double-sided tape.
*   Close the door.
*   Wake the screen.
*   **Hold Button B (Side) for 2 seconds.**
*   The screen will flash **White**.
*   **Done!** The sensor now knows this position is "Closed" and "Up".

---
 
## 📶 WiFi Configuration
 
If you need to change the WiFi network or don't want to hardcode credentials:
1.  Wake the screen with **Button A**.
2.  **Hold Button B (Side) for 5 seconds.**
3.  The screen will turn **Blue** and show "WIFI CONFIG MODE".
4.  Connect your phone/PC to the WiFi network: `M5-DoorSensor-XXXX`.
5.  Open `http://192.168.4.1` in your browser.
6.  Enter your WiFi credentials and click **Save**.
7.  The device will restart and connect to the new network.
 
---

## 🛠️ Configuration (Firmware)

### **WiFi & MQTT**
Run `idf.py menuconfig` -> `M5-DoorSensor Configuration` to set:
*   WiFi SSID & Password.
*   MQTT Broker IP, Port, User, Password.

### **Telegram Alerts**
1.  Get your **Bot Token** from [@BotFather](https://t.me/BotFather).
2.  Get your **Chat ID** from [@userinfobot](https://t.me/userinfobot).
3.  Run `idf.py menuconfig` -> `M5-DoorSensor Configuration` -> Enable Telegram and paste keys.

---

## 💻 PC Alert Script (Optional)
If you want instant pop-ups on your Windows PC:
1.  Install Python.
2.  Install dependencies: `pip install paho-mqtt plyer`
3.  Run the listener:
    ```resh
    python firmware/alert_listener.py
    ```

## ⚡ Flashing
```resh
idf.py build
idf.py -p COM8 flash monitor
```

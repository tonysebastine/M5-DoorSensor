# M5StickC Plus 2 - Smart Door Sensor

A high-performance codebase that transforms the **M5StickC Plus 2** into a smart, power-efficient Door Sensor.

## 🚀 Features
*   **Instant Alerts:** Detects door opening in **0.5 seconds**. Sends alerts via **MQTT** and **Telegram**.
*   **10+ Hour Battery:** Uses smart `Light Sleep` to last ~20-30 hours on a single charge.
*   **Modern UI:**
    *   **Green Box:** Door Closed.
    *   **Red Box:** Door Open.
    *   **Grey Box:** Disarmed (Silent).
    *   **Battery Bar:** Dynamic Green/Yellow/Red indicator at the bottom.
*   **Auto-Orientation:** The display automatically rotates to "Up" based on how you mount it.
*   **Arm/Disarm:** Toggle alerts on/off directly from the device.

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

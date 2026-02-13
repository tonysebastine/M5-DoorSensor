# Introduction
This guide shows you how to set up a local MQTT Broker on your Windows PC so your M5 Door Sensor can send alerts to it.

## 1. Install Mosquitto Broker
1.  Download the installer from [mosquitto.org](https://mosquitto.org/download/).
2.  Run the installer. **Note the installation folder** (usually `C:\Program Files\mosquitto`).
3.  Finish the installation.

## 2. Configure for External Access
By default, Mosquitto only listens to `localhost`. We need it to listen to your specific IP (`192.168.1.6`) so the M5Stick can talk to it.

1.  Open `C:\Program Files\mosquitto` (or where you installed it).
2.  Create a new text file named `my_config.conf`.
3.  Add these two lines to it:
    ```conf
    listener 1883
    allow_anonymous true
    ```
4.  (Optional) If you want a user/password, remove `allow_anonymous true` and see Mosquitto docs for creating a password file. For testing, `allow_anonymous true` is easiest.

## 3. Allow Through Firewall
1.  Search for **"Firewall & Network Protection"** in Windows.
2.  Select **"Advanced Settings"**.
3.  Click **"Inbound Rules"** -> **"New Rule..."**.
4.  Select **"Port"** -> TCP -> Specific local ports: **1883**.
5.  Select **"Allow the connection"**.
6.  Name it "Mosquitto MQTT".

## 4. Run the Broker manually (for testing)
1.  Open a Command Prompt (cmd) as Administrator.
2.  Navigate to the folder: `cd "C:\Program Files\mosquitto"`
3.  Run:
    ```cmd
    mosquitto -c my_config.conf -v
    ```
    *(The `-v` flag shows verbose logs so you can see when the M5Stick tries to connect)*.

## 5. Verify Setup
1.  Ensure your **M5Stick is connected to the SAME WiFi network** as your PC.
    *   Currently, likely your PC is on `192.168.1.x`, but your M5Stick was on `10.76.7.x`. This needs to be fixed!
2.  Update the **M5Stick Firmware**:
    *   Run `idf.py menuconfig`.
    *   Go to **M5-DoorSensor Configuration**.
    *   Change the **WiFi SSID** to match your PC's WiFi network name.
    *   Change the **WiFi Password** accordingly.
    *   Ensure **MQTT Broker IP** is set to `192.168.1.6` (I have updated this for you in `sdkconfig` already).
3.  Flash the device: `idf.py build flash monitor`.

If successful, you will see `New client connected from 192.168.1.x...` in your Mosquitto window.

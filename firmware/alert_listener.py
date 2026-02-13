import paho.mqtt.client as mqtt
import time
import json
import winsound  # Windows Sound
from plyer import notification  # Install with: pip install plyer paho-mqtt

# CONFIGURATION
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "m5door/tony/state"

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"Connected to MQTT Broker! Listening for Door Events on '{MQTT_TOPIC}'...")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)
        event = data.get("event", "UNKNOWN")
        battery = data.get("battery", 0) # Changed from battery_mv to battery based on JSON
        
        print(f"--- ALERT: Door is {event} --- (Battery: {battery}mV)")
        
        if event == "OPEN":
            try:
                winsound.Beep(1000, 500)
            except:
                pass
            
            try:
                notification.notify(
                    title='DOOR ALERT!',
                    message=f'The Door has been OPENED!\nBattery: {battery}mV',
                    app_name='M5 Door Monitor',
                    timeout=10
                )
            except Exception as e:
                print(f"Notification error: {e}")
            
    except Exception as e:
        print(f"Error parsing message: {e}")

if __name__ == "__main__":
    try:
        # Use explicit version 2 for modern paho-mqtt
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = on_connect
        client.on_message = on_message

        print(f"Connecting to {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"Connection Failed: {e}")
        print("Please check: 1. Broker is running. 2. IP is correct. 3. Firewall is open.")


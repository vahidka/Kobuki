import threading
import time
import json
import logging
from pypozyx import PozyxSerial, get_first_pozyx_serial_port, UWBSettings, EulerAngles, Coordinates
import paho.mqtt.client as mqtt

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# MQTT server details
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = 'pozyx/position'

def publish_position_data(client, pozyx):
    position = Coordinates()
    euler_angles = EulerAngles()
    while True:
        try:
            pozyx.getCoordinates(position)
            pozyx.getEulerAngles_deg(euler_angles)
            position_data = {'x': position.x, 'y': position.y, 'heading': euler_angles.heading}
            position_message = json.dumps(position_data)
            client.publish(MQTT_TOPIC, position_message)
            #logging.info(f"Published: {position_message}")
        except Exception as e:
            logging.error(f"Error obtaining or publishing position data: {e}")
        time.sleep(0.1)  # Adjust sleep time as needed

def on_connect(client, userdata, flags, rc):
    logging.info(f"Connected with result code {rc}")
    # Start publishing position data in a separate thread
    # Access the pozyx object passed as userdata
    pozyx = userdata['pozyx']
    thread = threading.Thread(target=publish_position_data, args=(client, pozyx))
    thread.daemon = True  # Daemon threads are abruptly stopped when the program exits
    thread.start()

def main():
    serial_port = get_first_pozyx_serial_port()
    if serial_port is not None:
        pozyx = PozyxSerial(serial_port)
        uwb_settings = UWBSettings()
        pozyx.getUWBSettings(uwb_settings)
        logging.info(uwb_settings)

        client = mqtt.Client(userdata={'pozyx': pozyx})  # Pass pozyx as part of userdata
        client.on_connect = on_connect
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Exiting")
            client.disconnect()
    else:
        logging.error("No Pozyx port was found")

if __name__ == "__main__":
    main()

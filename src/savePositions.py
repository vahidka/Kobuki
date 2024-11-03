import paho.mqtt.client as mqtt
import json
from datetime import datetime
import os

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    # Open the file in append mode
    with open('mqtt_data.txt', 'a') as f:
        # Write the topic and message to the file
        f.write('Timestamp: ' + str(datetime.now()) + '\n')
        f.write('Topic: ' + msg.topic + '\n')
        f.write(str(msg.payload) + '\n')

# Check if the file exists and remove it
if os.path.exists("mqtt_data.txt"):
    os.remove("mqtt_data.txt")
else:
    print("The file does not exist")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.on_message = on_message

# Connect to the MQTT broker
client.connect("192.168.2.100", 1883, 60)

# Subscribe to the topics
client.subscribe("robot/coordinates")
client.subscribe("pozyx/position")

# Blocking call that processes network traffic, dispatches callbacks and handles reconnecting.
client.loop_forever()
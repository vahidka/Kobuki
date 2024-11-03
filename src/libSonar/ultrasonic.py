import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt

# Set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.output(GPIO_TRIGGER, False)
time.sleep(2)  # Calibrating

def distance():

    # Send a pulse
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # Save start time
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # Save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # Time difference between start and arrival
    time_elapsed = stop_time - start_time
    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (time_elapsed * 34300) * 0.5

    return distance

def on_publish(client, userdata, result):
    #print("Data published \n")
    pass

# MQTT Settings
broker_address = "localhost"  # Use your MQTT broker address
port = 1883
topic = "sensor/distance"

client = mqtt.Client("P1")  # create new instance
client.on_publish = on_publish  # assign function to callback
client.connect(broker_address, port)  # connect to broker

# Initialize a list to store the readings
num_readings = 2  # Number of readings to average
readings = []

try:
    while True:
        dist = distance()
        # Add the new reading to the list and keep only the latest num_readings
        readings.append(dist)
        readings = readings[-num_readings:]
        
        # Calculate the average
        avg_dist = sum(readings) / len(readings)

        print("Measured Distance = %.1f cm, Average Distance = %.1f cm" % (dist, avg_dist))
        client.publish(topic, str(avg_dist))
        time.sleep(0.25)  # Publish at 4Hz

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()

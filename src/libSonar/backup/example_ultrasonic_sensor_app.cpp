#include "UltrasonicSensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    // Define the GPIO pins for the ultrasonic sensor (example: GPIO 17 for trigger, GPIO 18 for echo)
    int triggerPin = 18;
    int echoPin = 24;

    UltrasonicSensor sensor(triggerPin, echoPin);

    while (true) {
        double distance = sensor.getDistance();
        std::cout << "Distance: " << distance << " cm" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

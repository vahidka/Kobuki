#include "UltrasonicSensor.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin) {
    // Initialize the GPIO pins
    exportPin(triggerPin);
    exportPin(echoPin);
    setDirection(triggerPin, "out");
    setDirection(echoPin, "in");
}

UltrasonicSensor::~UltrasonicSensor() {
    unexportPin(triggerPin);
    unexportPin(echoPin);
}

double UltrasonicSensor::getDistance() {
    setPinValue(triggerPin, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Send a trigger pulse to start the measurement
    setPinValue(triggerPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    setPinValue(triggerPin, 0);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto timeout = std::chrono::milliseconds(500); // Timeout set to 500 milliseconds

    // Wait for the echo to go high
    while (getPinValue(echoPin) == 0) {
        if (std::chrono::high_resolution_clock::now() - start_time > timeout) {
            return -1; // Return -1 if timeout occurs (distance measurement unsuccessful)
        }
    }

    start_time = std::chrono::high_resolution_clock::now(); // Reset start time

    // Wait for the echo to go low
    while (getPinValue(echoPin) == 1) {
        if (std::chrono::high_resolution_clock::now() - start_time > timeout) {
            return -1; // Return -1 if timeout occurs (distance measurement unsuccessful)
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // Calculate the distance in centimeters
    double distance = pulse_duration * 0.034 / 2;
    return distance;
}

void UltrasonicSensor::exportPin(int pin) {
    std::ofstream exportFile("/sys/class/gpio/export");
    exportFile << pin;
    exportFile.close();
}

void UltrasonicSensor::unexportPin(int pin) {
    std::ofstream unexportFile("/sys/class/gpio/unexport");
    unexportFile << pin;
    unexportFile.close();
}

void UltrasonicSensor::setDirection(int pin, const std::string& direction) {
    std::string directionPath = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream directionFile(directionPath);
    directionFile << direction;
    directionFile.close();
}

void UltrasonicSensor::setPinValue(int pin, int value) {
    std::string valuePath = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream valueFile(valuePath);
    valueFile << value;
    valueFile.close();
}

int UltrasonicSensor::getPinValue(int pin) {
    std::string valuePath = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ifstream valueFile(valuePath);
    int value;
    valueFile >> value;
    valueFile.close();
    return value;
}

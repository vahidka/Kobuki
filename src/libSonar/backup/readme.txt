# Ultrasonic Sensor Library for Raspberry Pi

This C++ library provides a convenient interface for using an Ultrasonic Sensor (e.g., HC-SR04) with a Raspberry Pi. The library utilizes the sysfs interface for controlling GPIO pins, allowing you to measure distances using the ultrasonic sensor without the need for external libraries like WiringPi.

## Features

- Simple and clean C++ interface for Ultrasonic Sensor.
- Non-blocking measurement of distances.
- Supports Raspberry Pi GPIO pins.

## Prerequisites

- A Raspberry Pi (any model with GPIO pins).
- An Ultrasonic Sensor (e.g., HC-SR04).
- C++ compiler (e.g., g++).

## Usage

1. Clone or download this repository to your Raspberry Pi.

2. Include the `UltrasonicSensor.h` header in your C++ code:

    ```cpp
    #include "UltrasonicSensor.h"

    Compile your code along with UltrasonicSensor.cpp:
    g++ -o your_program your_source_file.cpp UltrasonicSensor.cpp -std=c++11

Customize the GPIO pin numbers in your source code to match your hardware setup.
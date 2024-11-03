#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
#include <string>

class UltrasonicSensor {
public:
    UltrasonicSensor(int triggerPin, int echoPin);
    ~UltrasonicSensor();
    double getDistance();

private:
    int triggerPin;
    int echoPin;
    void exportPin(int pin);
    void unexportPin(int pin);
    void setDirection(int pin, const std::string& direction);
    void setPinValue(int pin, int value);
    int getPinValue(int pin);
};

#endif // ULTRASONIC_SENSOR_H

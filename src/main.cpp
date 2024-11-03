//#include <string>
#include <fstream>
#include <sstream>
#include <csignal>
#include <iomanip>
#include "kobuki_manager.hpp"
#include "map_manager.hpp"
#include "motion_controller.hpp"
#include "mqtt/async_client.h"
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

bool shutdown_req = false;
int robot_id;

void signalHandler(int /* signum */)
{
    shutdown_req = true;
}

// Create an MQTT client instance
mqtt::async_client* mqtt_client;

void bumperHandler(const kobuki::BumperEvent &event) {
    cout << "sending BumperEvent" << endl;
    // Convert the bumper state to a string
    std::string bumper;
    switch (event.bumper) {
        case kobuki::BumperEvent::Bumper::Left:
            bumper = "Left";
            break;
        case kobuki::BumperEvent::Bumper::Center:
            bumper = "Center";
            break;
        case kobuki::BumperEvent::Bumper::Right:
            bumper = "Right";
            break;
        default:
            bumper = "Unknown";
    }
    // Map state enum to string representation
    std::string state = event.state == kobuki::BumperEvent::State::Pressed ? "Pressed" : "Released";
    // Create a JSON object
    nlohmann::json j;
    j["bumper"] = bumper;
    j["state"] = state;
    j["robot_id"] = robot_id;
    // Convert the JSON object to a string
    std::string message = j.dump();
    // Publish the message to the MQTT topic
    mqtt_client->publish("robot/bumper", message);
}

void cliffHandler(const kobuki::CliffEvent &event) {
    cout << "sending CliffEvent" << endl;
    // Map sensor enum to string representation
    std::string sensor;
    switch (event.sensor) {
        case kobuki::CliffEvent::Sensor::Left:
            sensor = "Left";
            break;
        case kobuki::CliffEvent::Sensor::Center:
            sensor = "Center";
            break;
        case kobuki::CliffEvent::Sensor::Right:
            sensor = "Right";
            break;
        default:
            sensor = "Unknown";
    }
    // Map state enum to string representation
    std::string state = event.state == kobuki::CliffEvent::State::Floor ? "Floor" : "Cliff";
    // Create a JSON object
    nlohmann::json j;
    j["state"] = state;
    j["sensor"] = sensor;
    j["robot_id"] = robot_id;
    // Convert the JSON object to a string
    std::string message = j.dump();
    // Publish the message to the MQTT topic
    mqtt_client->publish("robot/cliff", message);
}

// MQTT Client Initialization
void initialize_mqtt_client() {
    using namespace std::placeholders;
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    try {
        std::cout << "Connecting to the MQTT broker..." << std::endl;
        mqtt_client->connect(connOpts)->wait();
    } catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        // Handle connection failure
    }
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    // Read robot_id from config.json
    ifstream configFile("config.json");
    json config;
    configFile >> config;
    robot_id = config["robot_id"];
    string mqtt_broker_ip = config["mqtt_broker_ip"];
    configFile.close();
    mqtt_client = new mqtt::async_client("tcp://" + mqtt_broker_ip + ":1883", "MainClient_" + std::to_string(robot_id));
    initialize_mqtt_client();
    signal(SIGINT, signalHandler);
    ecl::MilliSleep sleep(1000);
    int ultrasonic_sensor_trigger_pin = 18;
    int ultrasonic_sensor_echo_pin = 24;
    KobukiManager kobuki_manager;
    kobuki_manager.setUserBumperEventCallBack(bumperHandler);
    kobuki_manager.setUserCliffEventCallBack(cliffHandler);
    MotionController motion_controller(kobuki_manager);
    try
    {
        while (!shutdown_req)
        {
            motion_controller.readSensors();
            //motion_controller.ShortcutBug2Algorithm();
            //motion_controller.Bug2Algorithm();
            //motion_controller.spiralAlgorithm();
            motion_controller.randomMovementAlgorithm();
            sleep(200);
        }
        motion_controller.stop();
    }
    catch (ecl::StandardException &e)
    {
        std::cout << e.what();
    }
    motion_controller.stop();
    sleep(300);
    return 0;
}
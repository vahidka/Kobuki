/**
 * @file kobuki_manager.hpp
 *
 * @brief High-level module for controlling a Kobuki robot.
 *
 * This file defines the KobukiManager class, which serves as a central
 * interface for interacting with a Kobuki robot. It includes functionalities
 * for processing sensor data, managing robot states, and handling various
 * robot events such as button presses, bumper hits, and cliff detections.
 * The class aims to provide a simplified and coherent interface for robot
 * control and management.
 *
**/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_MANAGER_HPP_
#define KOBUKI_MANAGER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <vector>
#include <ecl/geometry.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/command_line.hpp>
#include "kobuki_core/kobuki.hpp"
#include "kobuki_core/packets/core_sensors.hpp"

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef void (*userButtonEventCallBackType)(const kobuki::ButtonEvent &event);
typedef void (*userBumperEventCallBackType)(const kobuki::BumperEvent &event);
typedef void (*userCliffEventCallBackType)(const kobuki::CliffEvent &event);

/*****************************************************************************
** Classes
*****************************************************************************/

using namespace std;

class KobukiManager {
public:
    /**
    * Constructor.
    */
    KobukiManager();

    /**
    * Destructor.
    */
    ~KobukiManager();

    /**
    * @brief Move kobuki in given velocity.
    * @param longitudinal_velocity: (m/s)
    * @param rotational_velocity: (rad/s). Defaults to 0.0.
    */
    void move(double longitudinal_velocity, double rotational_velocity = 0.0);

    /**
    * @brief Rotate kobuki in given velocity
    * @param rotational_velocity: (rad/s)
    */
    void rotate(double rotational_velocity);

    /**
    * @brief Stop immediately
    */
    void stop();

    /**
    * @brief Get x and y coordinates
    * @return vector<double>: [x, y]
    */
    vector<double> getCoordinates();

    /**
    * @brief Get current angle in radians
    * @return double: radians
    */
    double getAngle();

    /**
    * @brief There are 3 bumper sensors with unique values.
    * Each bumper sensor adds up its own value to bumper state parameter.
    * @return int: BUMPERS: 0=NoHit, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
    */
    int getBumperState();

    /**
    * @brief There are 3 cliff sensors with unique values.
    * Each bumper sensor adds up its own value to cliff state parameter.
    * @return int: CLIFF: 0=NoHit, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
    */
    int getCliffState();

    /**
    * @brief Plays the sound sequence of the given value
    * @param int: Possible values:
    * kobuki::On = 0x0, kobuki::Off = 0x1, kobuki::Recharge = 0x2, kobuki::Button = 0x3,
    * kobuki::Error = 0x4, kobuki::CleaningStart = 0x5, kobuki::CleaningEnd = 0x6
    */
    void playSoundSequence(int x);

    /**
    * @brief Set function pointer as callback for Button Event
    */
    void setUserButtonEventCallBack (userButtonEventCallBackType func);

    /**
    * @brief Set function pointer as callback for Bumper Event
    */
    void setUserBumperEventCallBack (userBumperEventCallBackType func);

    /**
    * @brief Set function pointer as callback for Cliff Event
    */
    void setUserCliffEventCallBack (userCliffEventCallBackType func);

    /**
     * @brief Set the initial pose of the robot.
     * @param x The x coordinate of the initial pose.
     * @param y The y coordinate of the initial pose.
     * @param angle The heading angle of the initial pose.
     */
    void setInitialPose(double x, double y, double angle);

private:
    kobuki::Kobuki kobuki;
    ecl::linear_algebra::Vector3d pose; // x, y, heading
    double initial_heading;
    ecl::Slot<const kobuki::ButtonEvent&> slot_button_event;
    ecl::Slot<const kobuki::BumperEvent&> slot_bumper_event;
    ecl::Slot<const kobuki::CliffEvent&> slot_cliff_event;
    ecl::Slot<> slot_stream_data;
    kobuki::CoreSensors::Data data;
    ecl::TimeStamp start_time;

    /*
    * Function pointer callbacks:
    */
    userButtonEventCallBackType userButtonEventCallBack;
    userBumperEventCallBackType userBumperEventCallBack;
    userCliffEventCallBackType userCliffEventCallBack;

    /*
    * Prints to stdout with a timestamp
    */
    void customLogger(const std::string& message);

    /**
     * @brief Processes stream data from the robot's sensors.
     * This function is a callback that handles incoming sensor data,
     * updates internal states, and triggers appropriate responses.
     */
    void processStreamData();

    /**
     * @brief Handles button events from the Kobuki robot.
     * This function is called when there is a button event, processing
     * the event data and executing appropriate actions.
     * @param event The button event data.
     */
    void processButtonEvent(const kobuki::ButtonEvent &event);

    /**
     * @brief Handles bumper events from the Kobuki robot.
     * This function is triggered when the robot's bumper detects a collision,
     * allowing for appropriate response actions.
     * @param event The bumper event data.
     */
    void processBumperEvent(const kobuki::BumperEvent &event);

    /**
     * @brief Handles cliff events from the Kobuki robot.
     * Invoked when the robot detects a cliff, this function enables
     * the robot to take necessary safety measures.
     * @param event The cliff detection event data.
     */
    void processCliffEvent(const kobuki::CliffEvent &event);
};

#endif /* KOBUKI_MANAGER_HPP_ */

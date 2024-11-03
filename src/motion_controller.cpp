#include "motion_controller.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

// Use nlohmann::json namespace for convenience
using json = nlohmann::json;
using namespace std;

const string modeTopic("robot/mode");
const string stateTopic("robot/state");
const string coordinatesTopic("robot/coordinates");
const string obstacleTopic("robot/obstacle");

void buttonHandler(const kobuki::ButtonEvent &event) {
    cout << "motion controller: ButtonEvent" << endl;
    if (event.state == kobuki::ButtonEvent::Released) {
        if (event.button == kobuki::ButtonEvent::Button0) {
            cout << "Button 0 Released" << endl;
        }
    }
}

// Function to calculate the closest point on the line segment from point P to the line segment AB
std::pair<double, double> closestPointOnLine(double ax, double ay, double bx, double by, double px, double py) {
    double apx = px - ax;
    double apy = py - ay;
    double abx = bx - ax;
    double aby = by - ay;
    double ab2 = abx * abx + aby * aby;
    double ap_ab = apx * abx + apy * aby;
    double t = ap_ab / ab2;
    t = std::max(0.0, std::min(1.0, t)); // Clamp t to [0, 1]
    return {ax + abx * t, ay + aby * t};
}

MotionController::MotionController(KobukiManager& kobuki_manager)
    : mqtt_client("tcp://localhost:1883", "MotionControllerClient"),
      kobuki_manager(kobuki_manager) {
    // Read robot_id from config.json
    ifstream configFile("config.json");
    json config;
    configFile >> config;
    robot_id = config["robot_id"];
    string mqtt_broker_ip = config["mqtt_broker_ip"]; // Read the MQTT broker's IP address
    configFile.close();
    cout << "Robot ID: " << robot_id << endl;
    this->temp_target_x = this->target_x = 0.0;
    this->temp_target_y = this->target_y = 0.0;
    UWB_x = 0.0;
    UWB_y = 0.0;
    UWB_yaw = 0.0;
    start_time.stamp();
    robot_mode = CUSTOM_MODE;
    moving_state = GOAL_ACHIEVED;
    button0_flag = false;
    kobuki_manager.setUserButtonEventCallBack(buttonHandler);
    remote_client = new mqtt::async_client("tcp://" + mqtt_broker_ip + ":1883", "MotionControllerClient_" + std::to_string(robot_id));

    initialize_remote_mqtt_client();
    initialize_mqtt_client();
    // Wait for the first pozyx/position message
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this] { return pozyx_position_received; });
    kobuki_manager.setInitialPose(UWB_x, UWB_y, UWB_yaw);
    sendModeToMQTT();
    sendStateToMQTT();
}

MotionController::~MotionController() {
    map_manager.printMap(current_x, current_y);
    // Ensure to disconnect MQTT Client
    try {
        std::cout << "Disconnecting MQTT clients..." << std::endl;
        mqtt_client.disconnect();
        remote_client->disconnect();
        delete remote_client; // Free the dynamically allocated memory
    } catch (const mqtt::exception& exc) {
        std::cerr << "MQTT disconnection failed: " << exc.what() << std::endl;
    }
}

// MQTT Client Initialization
void MotionController::initialize_mqtt_client() {
    using namespace std::placeholders;
    mqtt_client.set_connected_handler(std::bind(&MotionController::on_connected, this, _1));
    mqtt_client.set_message_callback(std::bind(&MotionController::mqtt_message_arrived, this, _1));

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    try {
        std::cout << "Connecting to the MQTT broker..." << std::endl;
        mqtt_client.connect(connOpts)->wait();
    } catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        // Handle connection failure
    }
}

// MQTT Remote Client Initialization
void MotionController::initialize_remote_mqtt_client() {
    using namespace std::placeholders;
    remote_client->set_connected_handler(std::bind(&MotionController::on_remote_connected, this, _1));
    remote_client->set_message_callback(std::bind(&MotionController::remote_mqtt_message_arrived, this, _1));

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    try {
        std::cout << "Connecting to the remote MQTT broker..." << std::endl;
        remote_client->connect(connOpts)->wait();
    } catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        // Handle connection failure
    }
}

// MQTT Connection Success Handler
void MotionController::on_connected(const std::string& cause) {
    std::cout << "MQTT connection success" << std::endl;
    // Subscribe to topics
    mqtt_client.subscribe("sensor/distance", 1);
    mqtt_client.subscribe("pozyx/position", 1);
}

// MQTT Connection Success Handler
void MotionController::on_remote_connected(const std::string& cause) {
    std::cout << "MQTT connection success" << std::endl;
    // Subscribe to topics
    remote_client->subscribe("webUI/target", 1);
    remote_client->subscribe("webUI/stop", 1);
    remote_client->subscribe("webUI/move", 1);
    remote_client->subscribe("robot/obstacle", 1);
}

// MQTT Message Arrival Handler
void MotionController::mqtt_message_arrived(mqtt::const_message_ptr msg) {
    //std::cout << "Message arrived: " << msg->get_topic() << ": " << msg->to_string() << std::endl;
    // Handle message based on topic
    if (msg->get_topic() == "sensor/distance") {
        // Update based on distance message
        // Convert the payload to a string
        std::string payload = msg->to_string();
        double distance = 0.0;
        try {
            // Attempt to convert the string payload to a double
            distance = std::stod(payload);
            std::cout << "Distance received: " << distance << " units" << std::endl;
            // Now you can use the distance value as needed in your application
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: Could not convert the payload to a double." << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Out of range: The conversion resulted in an overflow or underflow." << std::endl;
        }
        if (pozyx_position_received) {
            checkDistance(distance);
        }
    } else if (msg->get_topic() == "pozyx/position") {
        // Update based on position message
        std::unique_lock<std::mutex> lock(mtx);
        try {
            // Convert the payload to a string and parse it as JSON
            auto payload = msg->to_string();
            auto j = json::parse(payload);

            // Extract the position data
            UWB_x = j["x"];
            UWB_y = j["y"];
            UWB_yaw = j["heading"];
            UWB_x = UWB_x * MM_TO_M;
            UWB_y = UWB_y * MM_TO_M;
            UWB_yaw = 2.0 * ecl::pi - (UWB_yaw  * (ecl::pi / 180.0));
            std::cout << "Position received - X: " << UWB_x << ", Y: " << UWB_y << ", heading: " << UWB_yaw << std::endl;
            // Now you can use x, y, heading as needed in your application
        } catch (json::parse_error& e) {
            std::cerr << "Parsing error: " << e.what() << '\n';
        } catch (json::type_error& e) {
            std::cerr << "Type error: " << e.what() << '\n';
        } catch (std::exception& e) {
            std::cerr << "Some other error: " << e.what() << '\n';
        }
        if (!pozyx_position_received) {
            // Set the flag and notify
            cv.notify_one();
            pozyx_position_received = true;
        }

        pozyx_counter++;
        if (pozyx_counter == 3) {
            sendCoordinatesToMQTT();
            pozyx_counter = 0;
        }
    }
}


// MQTT Message Arrival Handler
void MotionController::remote_mqtt_message_arrived(mqtt::const_message_ptr msg) {
    if (msg->get_topic() == "webUI/target") {
        // Update based on target message
        std::string payload = msg->to_string();
        try {
            // Attempt to convert the string payload to a double
            auto j = json::parse(payload);
            int id = j["robot_id"];
            if (id == robot_id) {
                double x = j["x"];
                double y = j["y"];
                temp_target_x = target_x = x;
                temp_target_y = target_y = y;
                robot_mode = GO_TO_GOAL_MODE;
                sendModeToMQTT();
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                kobuki_manager.playSoundSequence(0x5);
                std::cout << "New robot target received: X: " << temp_target_x << ", Y: " << temp_target_y << std::endl;
            }
        } catch (json::parse_error& e) {
            std::cerr << "Parsing error: " << e.what() << '\n';
        } catch (json::type_error& e) {
            std::cerr << "Type error: " << e.what() << '\n';
        } catch (std::exception& e) {
            std::cerr << "Some other error: " << e.what() << '\n';
        }
    } else if (msg->get_topic() == "webUI/stop") {
        std::string payload = msg->to_string();
        try {
            // Attempt to convert the string payload to a double
            auto j = json::parse(payload);
            int id = j["robot_id"];
            if (id == robot_id) {
                // Handle the stop command, e.g., stop the robot
                kobuki_manager.stop();
                std::cout << "Stop command received." << std::endl;
                robot_mode = CUSTOM_MODE;
                sendModeToMQTT();
                moving_state = GOAL_ACHIEVED;
                sendStateToMQTT();
                kobuki_manager.playSoundSequence(0x6);
            }
        } catch (json::parse_error& e) {
            std::cerr << "Parsing error: " << e.what() << '\n';
        } catch (json::type_error& e) {
            std::cerr << "Type error: " << e.what() << '\n';
        } catch (std::exception& e) {
            std::cerr << "Some other error: " << e.what() << '\n';
        }
    } else if (msg->get_topic() == "webUI/move") {
        // Update based on target message
        std::string payload = msg->to_string();
        try {
            // Attempt to convert the string payload to a double
            auto j = json::parse(payload);
            int id = j["robot_id"];
            if (id == robot_id) {
                double longitudinal_velocity = j["longitudinal_velocity"];
                double rotational_velocity = j["rotational_velocity"];
                robot_mode = CUSTOM_MODE;
                sendModeToMQTT();
                moving_state = GOAL_ACHIEVED;
                sendStateToMQTT();
                kobuki_manager.move(longitudinal_velocity * CM_TO_M, rotational_velocity);
                kobuki_manager.playSoundSequence(0x5);
                std::cout << "New robot movement received: longitudinal_velocity: " << longitudinal_velocity << 
                        ", rotational_velocity: " << rotational_velocity << std::endl;
            }
        } catch (json::parse_error& e) {
            std::cerr << "Parsing error: " << e.what() << '\n';
        } catch (json::type_error& e) {
            std::cerr << "Type error: " << e.what() << '\n';
        } catch (std::exception& e) {
            std::cerr << "Some other error: " << e.what() << '\n';
        }
    } else if (msg->get_topic() == "robot/obstacle") {
        // Update based on target message
        std::string payload = msg->to_string();
        try {
            // Attempt to convert the string payload to a double
            auto j = json::parse(payload);
            int id = j["robot_id"];
            if (id + 1 == robot_id && robot_mode == CUSTOM_MODE) { // Only process the message if from the previous robot_id
                double x = j["target_x"];
                double y = j["target_y"];
                temp_target_x = target_x = x;
                temp_target_y = target_y = y;
                robot_mode = GO_TO_GOAL_MODE;
                sendModeToMQTT();
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                kobuki_manager.playSoundSequence(0x5);
                std::cout << "New robot target received from the previous robot: X: " << 
                        x << ", Y: " << y << std::endl;
            }
        } catch (json::parse_error& e) {
            std::cerr << "Parsing error: " << e.what() << '\n';
        } catch (json::type_error& e) {
            std::cerr << "Type error: " << e.what() << '\n';
        } catch (std::exception& e) {
            std::cerr << "Some other error: " << e.what() << '\n';
        }
    }
}

void MotionController::sendModeToMQTT() {
    std::string mode;
    switch (robot_mode) {
        case CUSTOM_MODE:
            mode = "CUSTOM MODE";
            break;
        case GO_TO_GOAL_MODE:
            mode = "GO TO GOAL MODE";
            break;
        case WALL_FOLLOWING_MODE:
            mode = "WALL FOLLOWING MODE";
            break;
        default:
            mode = "UNKNOWN";
            break;
    }

    // Create a JSON object and add robot_id and mode
    nlohmann::json j;
    j["robot_id"] = robot_id;
    j["mode"] = mode;

    // Convert the JSON object to a string
    std::string payload = j.dump();

    // Publish the message with robot_id and mode to the MQTT broker
    mqtt::message_ptr msg = mqtt::make_message(modeTopic, payload);
    remote_client->publish(msg);
    std::cout << "Robot mode sent to MQTT: " << mode << std::endl;
}

void MotionController::sendCoordinatesToMQTT() {
    // Create a JSON object
    json j;
    j["x"] = (int) (kobuki_manager.getCoordinates()[0] * M_TO_MM);
    j["y"] = (int) (kobuki_manager.getCoordinates()[1] * M_TO_MM);
    // Convert heading to string
    std::string headingStr = std::to_string(kobuki_manager.getAngle() * (180.0 / ecl::pi));
    // Limit the number of characters in heading field to 6
    j["heading"] = headingStr.substr(0, 7);
    // Add Pozyx coordinates
    j["pozyx_x"] = (int)(UWB_x * M_TO_MM);
    j["pozyx_y"] = (int)(UWB_y * M_TO_MM);
    std::string pozyxHeadingStr = std::to_string(UWB_yaw * (180.0 / ecl::pi));
    j["pozyx_heading"] = pozyxHeadingStr.substr(0, 7);
    j["robot_id"] = robot_id;

    // Convert the JSON object to a string
    std::string payload = j.dump();

    // Publish the coordinates to the MQTT broker
    mqtt::message_ptr msg = mqtt::make_message(coordinatesTopic, payload);
    remote_client->publish(msg);
    //std::cout << "Robot coordinates sent to MQTT: " << payload << std::endl;
}

void MotionController::sendStateToMQTT() {
    std::string state;
    switch (moving_state) {
        case ADJUST_HEADING:
            state = "ADJUST HEADING";
            break;
        case GO_STRAIGHT:
            state = "GO STRAIGHT";
            break;
        case GOAL_ACHIEVED:
            state = "GOAL ACHIEVED";
            break;
        default:
            state = "UNKNOWN";
            break;
    }

    // Create a JSON object and add robot_id and state
    json j;
    j["robot_id"] = robot_id;
    j["state"] = state;

    // Convert the JSON object to a string
    std::string payload = j.dump();

    // Publish the message with robot_id and state to the MQTT broker
    mqtt::message_ptr msg = mqtt::make_message(stateTopic, payload);
    std::cout << "Robot state sent to MQTT: " << state << std::endl;
}

void MotionController::readSensors() {
    // Get the most recent coordinates from UWB and Kobuki, and calculate the average
    vector<double> kobuki_coordinates = kobuki_manager.getCoordinates();
    cout << ecl::green << "kobuki_coordinates: [x: " << kobuki_coordinates[0] 
            << ", y: " << kobuki_coordinates[1] << ecl::reset << endl;
    current_x = (0.2 * UWB_x + 0.8 * kobuki_coordinates[0]);
    current_y = (0.2 * UWB_y + 0.8 * kobuki_coordinates[1]);
    current_yaw = kobuki_manager.getAngle();
    //current_yaw = UWB_yaw;
    if (robot_mode == CUSTOM_MODE) {
        return;
    }
    cout << ecl::green << "TimeStamp:" << double(ecl::TimeStamp() - start_time) << 
            ". [x: " << current_x << ", y: " << current_y;
    cout << ", heading: " << current_yaw << "]. Bumper State: " << kobuki_manager.getBumperState() << ". Cliff State: " <<
            kobuki_manager.getCliffState() << ecl::reset << endl;

    if (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0) {
        map_manager.updateMapPolar(ROBOT_RADIUS, UWB_yaw, current_x, current_y, 1);
        map_manager.printMap(current_x, current_y);
    }
    setObstacleFlags();
}

void MotionController::spiralAlgorithm() {
    double W, radius_value;
    for (int i=30; i<=500; i=i+1) {
        readSensors();
        radius_value = i;
        W=0.2/(radius_value/100.0);
        //W=L*R (R is divided by 100 to convert meters to cm)
        //(The unit of output for W is radian so there is no need to convert again in the setBaseControl function)
        std::cout << "W = " << W << std::endl;
        kobuki_manager.move(0.2, W);
        map_manager.sendGridToMQTT();
        cout << "TimeStamp:" << double(ecl::TimeStamp() - start_time) << ". Covered area: " 
        << map_manager.calculateZeroPercentageAroundOrigin(1.5) << " percent" << endl;
        ecl::MilliSleep sleep(1000);
        sleep(500+i*4);
    }
}

void MotionController::randomMovementAlgorithm() {
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> moveDurationDist(2.0, 6.0); // Duration for forward movement between 2 to 6 seconds
    std::uniform_real_distribution<double> rotateAngleDist(0.5, 1.0); // Rotation duration between 0.5 to 1.0 seconds
    std::uniform_int_distribution<int> moveChoiceDist(0, 1); // 0 for move forward, 1 for rotate

    while (true) {  // Continuously run within a control loop or until a stop condition is met
        double moveDuration = moveDurationDist(generator);
        double rotateDuration = rotateAngleDist(generator);
        int moveChoice = moveChoiceDist(generator);
        map_manager.sendGridToMQTT();
        readSensors();
        cout << "TimeStamp:" << double(ecl::TimeStamp() - start_time) << ". Covered area: " 
        << map_manager.calculateZeroPercentageAroundOrigin(1.5) << " percent" << endl;
        if (moveChoice == 0) {
            // Move forward
            kobuki_manager.move(FORWARD_SPEED, 0.0);
            cout << "Random Movement: Moving forward for " << moveDuration << " seconds." << endl;
            ecl::MilliSleep sleep(100);
            int steps = (int)(1000 * moveDuration / 100);
            for (int i = 0; i < steps; ++i) {
                sleep();
                readSensors();
                if (kobuki_manager.getBumperState() != 0) {
                    // Bumper hit, perform immediate rotation
                    double reactionRotateAngle = rotateAngleDist(generator) * (generator()%2 ? 1 : -1);  // Randomly decide direction
                    while (kobuki_manager.getBumperState() != 0) {
                        kobuki_manager.move(0.0, reactionRotateAngle * FAST_ROTATION_SPEED); // Rotate in place
                        cout << "Bumper Hit: Rotating immediately by " << reactionRotateAngle << " radians." << endl;
                        ecl::MilliSleep reactionSleep(1000 * rotateDuration); // Continue rotating for the rotate duration
                        reactionSleep();
                    }
                    break;  // Exit the current movement loop
                }
            }
        } else {
            // Rotate
            kobuki_manager.move(0.0, FAST_ROTATION_SPEED); // Start rotation
            cout << "Random Movement: Rotating for " << rotateDuration << " seconds." << endl;
            ecl::MilliSleep sleep(1000 * rotateDuration); // Complete the rotation without interruption
            sleep();
        }

        // Stop the robot after movement or rotation completes or after a bumper reaction
        kobuki_manager.stop();
        cout << "Stopping for a brief moment." << endl;
        ecl::MilliSleep(1000); // Pause for a second before the next move
    }
}



void MotionController::Bug2Algorithm() {
    double longitudinal_velocity = 0.0;
    double rotational_velocity = 0.0;

    map_manager.sendGridToMQTT();
    cout << "TimeStamp:" << double(ecl::TimeStamp() - start_time) << ". Covered area: " 
    << map_manager.calculateZeroPercentageAroundOrigin(1.5) << " percent" << endl;
    double position_error = sqrt(
                pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

    // Start of BUG 2 Algorithm
    if (robot_mode == GO_TO_GOAL_MODE) { // "go to target mode"
        if ((moving_state == GO_STRAIGHT) && (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0
                 || right_front_obstacle || front_obstacle || left_front_obstacle)) { // HIT!
            kobuki_manager.move(longitudinal_velocity, rotational_velocity);
            robot_mode = WALL_FOLLOWING_MODE; // wall following mode
            sendModeToMQTT();
            // save hit point coordinates:
            hit_x = current_x;
            hit_y = current_y;
            distance_to_goal_from_hit_point = sqrt((
                    pow(target_x - hit_x, 2)) +
                    (pow(target_y - hit_y, 2)));
            map_manager.printMap(current_x, current_y);
            cout << "hit_x: " << hit_x << " hit_y: " << hit_y << " distance_to_goal_from_hit_point: " 
                    << distance_to_goal_from_hit_point << endl;
                    cout << "GO_TO_GOAL_MODE, GO_STRAIGHT -> WALL_FOLLOWING_MODE robot_mode: " 
                    << robot_mode << " WALL_FOLLOWING_MODE, moving_mode: " << moving_state << endl;
            // stop and switch to wall mode
            sendObstacleEventToMQTT(target_x, target_y);
            return;
        }

        if (moving_state == ADJUST_HEADING) { // ADJUST HEADING
            double yaw_error = getYawError(current_x, current_y, current_yaw, target_x, target_y);
            // Adjust heading if heading is not good enough
            if (fabs(yaw_error) > yaw_precision) {
                if (yaw_error > 0.5) {
                    // Turn left (counterclockwise)
                    rotational_velocity = FAST_ROTATION_SPEED;
                } else if (yaw_error > 0) { // very close, rotate slowly
                    // Turn left (counterclockwise)
                    longitudinal_velocity = FORWARD_SPEED * 0.2;
                    rotational_velocity = ROTATION_SPEED;
                } else if (yaw_error < -0.5) {
                    // Turn right (clockwise)
                    rotational_velocity = -FAST_ROTATION_SPEED;
                } else { // very close, rotate slowly
                    // Turn right (clockwise)
                    longitudinal_velocity = FORWARD_SPEED * 0.2;
                    rotational_velocity = -ROTATION_SPEED;
                }
            } else {
                kobuki_manager.stop();
                moving_state = GO_STRAIGHT;
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                cout << "ADJUST_HEADING -> GO_STRAIGHT robot_mode: " << robot_mode 
                        << ", moving_mode: GO_STRAIGHT " << moving_state << endl;
                ecl::MilliSleep sleep(1000);
                sleep(200);
            }
        } else if (moving_state == GO_STRAIGHT) { // GO STRAIGHT
            if (position_error > 0.15) {
                if (isObstacleInFront) {
                    if (robot_id % 2 == 0) { // Even robot_id is right wall follower
                        // turn right and move forward slowly
                        rotational_velocity = -ROTATION_SPEED * 0.2;
                    } else { // Odd robot_id is left wall follower
                        // turn left and move forward slowly
                        rotational_velocity = ROTATION_SPEED * 0.2;
                    }
                }
                longitudinal_velocity = FORWARD_SPEED;
                // How far off is the current heading in radians?
                double yaw_error = getYawError(current_x, current_y, current_yaw, target_x, target_y);
                cout << "yaw_error: " << yaw_error << " position_error: " << position_error << endl;

                // Adjust heading if heading is not good enough
                if (fabs(yaw_error) > yaw_precision + ecl::pi * 0.05) {
                    moving_state = ADJUST_HEADING; // ADJUST HEADING
                    sendStateToMQTT();
                    map_manager.printMap(current_x, current_y);
                    cout << "GO_STRAIGHT -> ADJUST_HEADING robot_mode: " << robot_mode 
                            << ", moving_mode: " << moving_state << endl;
                }
            } else {   // If distance to target is smaller than 20cm
                moving_state = GOAL_ACHIEVED; // finish successfully
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                cout << "GO_STRAIGHT -> GOAL_ACHIEVED robot_mode: " << robot_mode 
                        << ", moving_mode: " << moving_state << endl;
                //kobuki_manager.setInitialPose(UWB_x, UWB_y, UWB_yaw);
                kobuki_manager.stop();
                kobuki_manager.playSoundSequence(0x6);
                cout << "DONE!" << endl;
            }
        } else if (moving_state == GOAL_ACHIEVED) { // GOAL_ACHIEVED
            /*TODO: use target list or buttons for new targets in the future*/
            if (button0_flag) {
                cout << "B0 pressed!!!" << endl;
                target_x = 0.0;
                target_y = 0.0;
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                kobuki_manager.playSoundSequence(0x5);
                cout << "GOAL_ACHIEVED -> ADJUST_HEADING Robot mode: " 
                        << robot_mode << ", moving mode: " << moving_state << endl;
                button0_flag = false;
            }
        }
    } else if (robot_mode == WALL_FOLLOWING_MODE) { // "wall following mode"
        if (position_error < 0.15) {
            robot_mode = GO_TO_GOAL_MODE; // "go to goal mode"
            moving_state = GOAL_ACHIEVED; // finish successfully
            sendStateToMQTT();
            map_manager.printMap(current_x, current_y);
            cout << "GO_STRAIGHT -> GOAL_ACHIEVED robot_mode: " << robot_mode 
                    << ", moving_mode: " << moving_state << endl;
            //kobuki_manager.setInitialPose(UWB_x, UWB_y, UWB_yaw);
            kobuki_manager.stop();
            kobuki_manager.playSoundSequence(0x6);
            cout << "DONE!" << endl;
            return;
        }
        // Distance to the line:
        double a = hit_y - target_y;
        double b = target_x - hit_x;
        double c = hit_x * target_y - target_x * hit_y;
        double distance_to_target_line = abs(a * current_x + b * current_y + c) / sqrt(a * a + b * b);
        if (distance_to_target_line < 0.10) { // If we hit the start-goal line again?
            // Is the leave point closer to the goal than the hit point?
            // If yes, go to goal. (Should be at least 20cm closer in order to avoid looping)
            double distance_to_goal_from_crossing_point = sqrt(
                pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            if ((distance_to_goal_from_hit_point - distance_to_goal_from_crossing_point) > 0.2) {
                cout << "HIT the GOAL LINE! ";
                map_manager.printMap(current_x, current_y);
                robot_mode = GO_TO_GOAL_MODE; // "go to goal mode"
                sendModeToMQTT();
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                cout << "WALL_FOLLOWING_MODE -> GO_TO_GOAL_MODE, ADJUST_HEADING Robot mode: " 
                        << robot_mode << ", moving mode: " << moving_state << endl;
                kobuki_manager.move(longitudinal_velocity, rotational_velocity);
                return;
            }
        }
        if (robot_id % 2 == 0) { // Even robot_id is right wall follower
            // BUMPERS: 0, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
            if (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0 || center_obstacle) {
                // move backwards
                cout << "Moving backwards" << endl;
                rotational_velocity = 0.0;
                longitudinal_velocity = -FORWARD_SPEED;
            } else if (front_obstacle || right_front_obstacle) {
                // turn right and move forward slowly
                rotational_velocity = -FAST_ROTATION_SPEED;
            } else if (left_front_obstacle) {
                // turn right and move forward slowly
                longitudinal_velocity = FORWARD_SPEED * 0.5;
                rotational_velocity = -FAST_ROTATION_SPEED;
            } else if (left_obstacle) {
                // move straight to follow the wall
                longitudinal_velocity = FORWARD_SPEED;
            } else if (kobuki_manager.getBumperState() == 0 || kobuki_manager.getCliffState() != 0) {
                // turn left and move forward slowly
                longitudinal_velocity = FORWARD_SPEED;
                rotational_velocity = FAST_ROTATION_SPEED;
            }
        } else { // Odd robot_id is left wall follower
            // BUMPERS: 0, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
            if (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0 || center_obstacle) {
                // move backwards
                cout << "Moving backwards" << endl;
                rotational_velocity = 0.0;
                longitudinal_velocity = -FORWARD_SPEED;
                //rotational_velocity = ROTATION_SPEED * 0.5;
            } else if (front_obstacle || left_front_obstacle) {
                // turn left and move forward slowly
                rotational_velocity = FAST_ROTATION_SPEED;
            } else if (right_front_obstacle) {
                // turn left and move forward slowly
                longitudinal_velocity = FORWARD_SPEED * 0.5;
                rotational_velocity = FAST_ROTATION_SPEED;
            } else if (right_obstacle) {
                // move straight to follow the wall
                longitudinal_velocity = FORWARD_SPEED;
            } else if (kobuki_manager.getBumperState() == 0 || kobuki_manager.getCliffState() != 0) {
                // turn right and move forward slowly
                longitudinal_velocity = FORWARD_SPEED;
                rotational_velocity = -FAST_ROTATION_SPEED;
            }
        }
    }
    kobuki_manager.move(longitudinal_velocity, rotational_velocity);
    return;
}

void MotionController::ShortcutBug2Algorithm() {
    double longitudinal_velocity = 0.0;
    double rotational_velocity = 0.0;

    double position_error = sqrt(
                pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

    // Start of BUG 2 Algorithm
    if (robot_mode == GO_TO_GOAL_MODE) { // "go to target mode"
        if ((moving_state == GO_STRAIGHT) && (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0
                 || right_front_obstacle || front_obstacle || left_front_obstacle)) { // HIT!
            kobuki_manager.move(longitudinal_velocity, rotational_velocity);
            robot_mode = WALL_FOLLOWING_MODE; // wall following mode
            sendModeToMQTT();
            // save hit point coordinates:
            hit_x = current_x;
            hit_y = current_y;
            distance_to_goal_from_hit_point = sqrt((
                    pow(target_x - hit_x, 2)) +
                    (pow(target_y - hit_y, 2)));
            map_manager.printMap(current_x, current_y);
            cout << "hit_x: " << hit_x << " hit_y: " << hit_y << " distance_to_goal_from_hit_point: " 
                    << distance_to_goal_from_hit_point << endl;
                    cout << "GO_TO_GOAL_MODE, GO_STRAIGHT -> WALL_FOLLOWING_MODE robot_mode: " 
                    << robot_mode << " WALL_FOLLOWING_MODE, moving_mode: " << moving_state << endl;
            // stop and switch to wall mode
            sendObstacleEventToMQTT(target_x, target_y);
            return;
        }

        if (moving_state == ADJUST_HEADING) { // ADJUST HEADING
            double yaw_error = getYawError(current_x, current_y, current_yaw, target_x, target_y);
            // Adjust heading if heading is not good enough
            if (fabs(yaw_error) > yaw_precision) {
                if (yaw_error > 0.5) {
                    // Turn left (counterclockwise)
                    rotational_velocity = FAST_ROTATION_SPEED;
                } else if (yaw_error > 0) { // very close, rotate slowly
                    // Turn left (counterclockwise)
                    longitudinal_velocity = FORWARD_SPEED * 0.2;
                    rotational_velocity = ROTATION_SPEED;
                } else if (yaw_error < -0.5) {
                    // Turn right (clockwise)
                    rotational_velocity = -FAST_ROTATION_SPEED;
                } else { // very close, rotate slowly
                    // Turn right (clockwise)
                    longitudinal_velocity = FORWARD_SPEED * 0.2;
                    rotational_velocity = -ROTATION_SPEED;
                }
            } else {
                kobuki_manager.stop();
                moving_state = GO_STRAIGHT;
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                cout << "ADJUST_HEADING -> GO_STRAIGHT robot_mode: " << robot_mode 
                        << ", moving_mode: GO_STRAIGHT " << moving_state << endl;
                ecl::MilliSleep sleep(1000);
                sleep(200);
            }
        } else if (moving_state == GO_STRAIGHT) { // GO STRAIGHT
            if (position_error > 0.25) {
                if (isObstacleInFront) {
                    if (robot_id % 2 == 0) { // Even robot_id is right wall follower
                        // turn right and move forward slowly
                        rotational_velocity = -ROTATION_SPEED * 0.2;
                    } else { // Odd robot_id is left wall follower
                        // turn left and move forward slowly
                        rotational_velocity = ROTATION_SPEED * 0.2;
                    }
                }
                longitudinal_velocity = FORWARD_SPEED;
                // How far off is the current heading in radians?
                double yaw_error = getYawError(current_x, current_y, current_yaw, target_x, target_y);
                cout << "yaw_error: " << yaw_error << " position_error: " << position_error << endl;

                // Adjust heading if heading is not good enough
                if (fabs(yaw_error) > yaw_precision + ecl::pi * 0.33) {
                    moving_state = ADJUST_HEADING; // ADJUST HEADING
                    sendStateToMQTT();
                    map_manager.printMap(current_x, current_y);
                    cout << "GO_STRAIGHT -> ADJUST_HEADING robot_mode: " << robot_mode 
                            << ", moving_mode: " << moving_state << endl;
                }
            } else {   // If distance to target is smaller than 20cm
                moving_state = GOAL_ACHIEVED; // finish successfully
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                cout << "GO_STRAIGHT -> GOAL_ACHIEVED robot_mode: " << robot_mode 
                        << ", moving_mode: " << moving_state << endl;
                //kobuki_manager.setInitialPose(UWB_x, UWB_y, UWB_yaw);
                kobuki_manager.stop();
                kobuki_manager.playSoundSequence(0x6);
                cout << "DONE!" << endl;
            }
        } else if (moving_state == GOAL_ACHIEVED) { // GOAL_ACHIEVED
            /*TODO: use target list or buttons for new targets in the future*/
            if (button0_flag) {
                cout << "B0 pressed!!!" << endl;
                target_x = 0.0;
                target_y = 0.0;
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                map_manager.printMap(current_x, current_y);
                kobuki_manager.playSoundSequence(0x5);
                cout << "GOAL_ACHIEVED -> ADJUST_HEADING Robot mode: " 
                        << robot_mode << ", moving mode: " << moving_state << endl;
                button0_flag = false;
            }
        }
    } else if (robot_mode == WALL_FOLLOWING_MODE) { // "wall following mode"
        if (position_error < 0.25) {
            robot_mode = GO_TO_GOAL_MODE; // "go to goal mode"
            moving_state = GOAL_ACHIEVED; // finish successfully
            sendStateToMQTT();
            map_manager.printMap(current_x, current_y);
            cout << "GO_STRAIGHT -> GOAL_ACHIEVED robot_mode: " << robot_mode 
                    << ", moving_mode: " << moving_state << endl;
            //kobuki_manager.setInitialPose(UWB_x, UWB_y, UWB_yaw);
            kobuki_manager.stop();
            kobuki_manager.playSoundSequence(0x6);
            cout << "DONE!" << endl;
            return;
        }
        // Distance to the line:
        double a = hit_y - target_y;
        double b = target_x - hit_x;
        double c = hit_x * target_y - target_x * hit_y;
        double distance_to_target_line = abs(a * current_x + b * current_y + c) / sqrt(a * a + b * b);
        // Check if the robot is closer to the target than the hit point
        double distance_to_goal_from_current_position = sqrt(
            pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

        if (distance_to_target_line < 0.10) { // If we hit the start-goal line again?
            // Is the leave point closer to the goal than the hit point?
            // If yes, go to goal. (Should be at least 20cm closer in order to avoid looping)
            double distance_to_goal_from_crossing_point = sqrt(
                pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            if ((distance_to_goal_from_hit_point - distance_to_goal_from_crossing_point) > 0.2) {
                cout << "HIT the GOAL LINE! ";
                map_manager.printMap(current_x, current_y);
                robot_mode = GO_TO_GOAL_MODE; // "go to goal mode"
                sendModeToMQTT();
                moving_state = ADJUST_HEADING;
                sendStateToMQTT();
                cout << "WALL_FOLLOWING_MODE -> GO_TO_GOAL_MODE, ADJUST_HEADING Robot mode: " 
                        << robot_mode << ", moving mode: " << moving_state << endl;
                kobuki_manager.move(longitudinal_velocity, rotational_velocity);
                return;
            }
        }
        cout << "distance_to_goal_from_hit_point: " << distance_to_goal_from_hit_point << endl;
        cout << "distance_to_goal_from_current_position: " << distance_to_goal_from_current_position << endl;
        cout << "diff: " << (distance_to_goal_from_hit_point - distance_to_goal_from_current_position) << endl;
        if (distance_to_goal_from_hit_point - distance_to_goal_from_current_position > 0.25) {
            // Calculate the closest point on the target line to the current position
            std::pair<double, double> closest_point = closestPointOnLine(
                hit_x, hit_y, target_x, target_y, current_x, current_y);
            // Move towards the closest point on the target line
            double temp_target_x = closest_point.first;
            double temp_target_y = closest_point.second;
            cout << "temp_target_x " << temp_target_x << " temp_target_y " << temp_target_y << endl;

            if (noObstaclesInPath(temp_target_x, temp_target_y)) {
                // Adjust heading and move towards the closest point on the target line
                double yaw_error = getYawError(current_x, current_y, current_yaw, temp_target_x, temp_target_y);
                if (fabs(yaw_error) > yaw_precision) {
                    // Adjust heading
                    if (yaw_error > 0) {
                        rotational_velocity = ROTATION_SPEED;
                    } else {
                        rotational_velocity = -ROTATION_SPEED;
                    }
                } else {
                    // Move straight towards the closest point on the target line
                    longitudinal_velocity = FORWARD_SPEED;
                }
                cout << "Moving towards the line!" << endl;
                kobuki_manager.move(longitudinal_velocity, rotational_velocity);
                return;
            }
        }

        if (robot_id % 2 == 0) { // Even robot_id is right wall follower
            // BUMPERS: 0, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
            if (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0 || center_obstacle) {
                // move backwards
                cout << "Moving backwards" << endl;
                rotational_velocity = 0.0;
                longitudinal_velocity = -FORWARD_SPEED;
            } else if (front_obstacle || right_front_obstacle) {
                // turn right and move forward slowly
                rotational_velocity = -FAST_ROTATION_SPEED;
            } else if (left_front_obstacle) {
                // turn right and move forward slowly
                longitudinal_velocity = FORWARD_SPEED * 0.5;
                rotational_velocity = -FAST_ROTATION_SPEED;
            } else if (left_obstacle) {
                // move straight to follow the wall
                longitudinal_velocity = FORWARD_SPEED;
            } else if (kobuki_manager.getBumperState() == 0 || kobuki_manager.getCliffState() != 0) {
                // turn left and move forward slowly
                longitudinal_velocity = FORWARD_SPEED;
                rotational_velocity = FAST_ROTATION_SPEED;
            }
        } else { // Odd robot_id is left wall follower
            // BUMPERS: 0, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
            if (kobuki_manager.getBumperState() != 0 || kobuki_manager.getCliffState() != 0 || center_obstacle) {
                // move backwards
                cout << "Moving backwards" << endl;
                rotational_velocity = 0.0;
                longitudinal_velocity = -FORWARD_SPEED;
                //rotational_velocity = ROTATION_SPEED * 0.5;
            } else if (front_obstacle || left_front_obstacle) {
                // turn left and move forward slowly
                rotational_velocity = FAST_ROTATION_SPEED;
            } else if (right_front_obstacle) {
                // turn left and move forward slowly
                longitudinal_velocity = FORWARD_SPEED * 0.5;
                rotational_velocity = FAST_ROTATION_SPEED;
            } else if (right_obstacle) {
                // move straight to follow the wall
                longitudinal_velocity = FORWARD_SPEED;
            } else if (kobuki_manager.getBumperState() == 0 || kobuki_manager.getCliffState() != 0) {
                // turn right and move forward slowly
                longitudinal_velocity = FORWARD_SPEED;
                rotational_velocity = -FAST_ROTATION_SPEED;
            }
        }
    }
    kobuki_manager.move(longitudinal_velocity, rotational_velocity);
    return;
}

void MotionController::checkDistance(double sensor_distance) {
    double distance = sensor_distance * CM_TO_M;
    cout << "Sensor Distance: " << distance << "m." << endl;
    if (distance > 0.02 && distance < 0.6) {
        //map_manager.updateMapPolar(distance + ROBOT_RADIUS, UWB_yaw, current_x, current_y, 1);
    }
    map_manager.updateMap(current_x, current_y, 0);
}

void MotionController::stop() {
    kobuki_manager.stop();
}

void MotionController::setObstacleFlags() {
    double x, y; // row, column
    center_obstacle = front_obstacle = right_front_obstacle = right_obstacle = left_front_obstacle = left_obstacle = false;
    // Check the obstacles around the robot. Check 3x3 grid for each position
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            // check center
            x = current_x + (i * GRID_SIZE) * cos(current_yaw);
            y = current_y + (j * GRID_SIZE) * sin(current_yaw);
            if (map_manager.checkMap(x, y)) {
                center_obstacle = true;
            }
            // check front
            x = current_x + (ROBOT_RADIUS + i * GRID_SIZE) * cos(current_yaw);
            y = current_y + (ROBOT_RADIUS + j * GRID_SIZE) * sin(current_yaw);
            if (map_manager.checkMap(x, y)) {
                front_obstacle = true;
            }
            // check right_front
            x = current_x + (ROBOT_RADIUS + i * GRID_SIZE) * cos(current_yaw - ecl::pi/5);
            y = current_y + (ROBOT_RADIUS + j * GRID_SIZE) * sin(current_yaw - ecl::pi/5);
            if (map_manager.checkMap(x, y)) {
                right_front_obstacle = true;
            }
            // check right
            x = current_x + (ROBOT_RADIUS + i * GRID_SIZE) * cos(current_yaw - ecl::pi*0.5);
            y = current_y + (ROBOT_RADIUS + j * GRID_SIZE) * sin(current_yaw - ecl::pi*0.5);
            if (map_manager.checkMap(x, y)) {
                right_obstacle = true;
            }
            // check left_front
            x = current_x + (ROBOT_RADIUS + i * GRID_SIZE) * cos(current_yaw + ecl::pi/5);
            y = current_y + (ROBOT_RADIUS + j * GRID_SIZE) * sin(current_yaw + ecl::pi/5);
            if (map_manager.checkMap(x, y)) {
                left_front_obstacle = true;
            }
            // check left
            x = current_x + (ROBOT_RADIUS + i * GRID_SIZE) * cos(current_yaw + ecl::pi*0.5);
            y = current_y + (ROBOT_RADIUS + j * GRID_SIZE) * sin(current_yaw + ecl::pi*0.5);
            if (map_manager.checkMap(x, y)) {
                left_obstacle = true;
            }
        }
    }

    isObstacleInFront = false;
    // check the path in front of the robot
    for (int i = 1; i < 8; i++) {
        x = current_x + i * 2 * ROBOT_RADIUS * cos(current_yaw);
        y = current_y + i * 2 * ROBOT_RADIUS * sin(current_yaw);
        if (map_manager.checkMap(x, y)) {
            isObstacleInFront = true;
            break;
        }
    }
    cout << "OBSTACLES:    " <<  isObstacleInFront << endl;
    cout << "OBSTACLES:    " <<  front_obstacle << endl;
    cout << "OBSTACLES:  " << left_front_obstacle << "   " << right_front_obstacle << endl;
    cout << "OBSTACLES:" << left_obstacle << "   " << center_obstacle << "   " << right_obstacle << endl;
}

double MotionController::getYawError(double current_x, double current_y, 
        double current_yaw, double target_x, double target_y) {
    double desired_yaw = atan2(
        target_y - current_y,
        target_x - current_x);
    // How far off is the current heading in radians?
    double yaw_error = desired_yaw - current_yaw;
    return ecl::wrap_angle(yaw_error);
}

void MotionController::sendObstacleEventToMQTT(double target_x, double target_y) {
    map_manager.printMap(current_x, current_y);
    json obstacle_event;
    obstacle_event["event"] = "obstacle";
    obstacle_event["target_x"] = target_x;
    obstacle_event["target_y"] = target_y;
    obstacle_event["robot_id"] = robot_id;
    std::string obstacle_event_str = obstacle_event.dump();
    try {
        mqtt::message_ptr msg = mqtt::make_message(obstacleTopic, obstacle_event_str);
        remote_client->publish(msg);
    } catch (const mqtt::exception& exc) {
        std::cerr << "Failed to send obstacle event to MQTT: " << exc.what() << std::endl;
    }
}

bool MotionController::noObstaclesInPath(double target_x, double target_y) {
    // Calculate the number of segments to discretize the line
    int num_segments = std::max(std::abs(target_x - current_x), std::abs(target_y - current_y)) / GRID_SIZE / 2;
    // Iterate over each segment
    for (int i = 0; i <= num_segments; ++i) {
        // Calculate the coordinates of the current segment
        double x = current_x + i * (target_x - current_x) / num_segments;
        double y = current_y + i * (target_y - current_y) / num_segments;
        for (int r = -2; r < 3; ++r) {
            for (int c = -2; c < 3; ++c) {
                // Check if there is an obstacle at the current segment
                if (map_manager.checkMap(x+r*GRID_SIZE, y+c*GRID_SIZE)) {
                    cout << "num_segments: " << num_segments << " x " << x << " r " << r << " y " << y << " c " << c << endl;
                    return false; // Obstacle detected
                }
            }
        }
    }
    return true; // No obstacles detected
}

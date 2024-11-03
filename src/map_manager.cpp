#include "map_manager.hpp"
#include <ecl/geometry.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>

using json = nlohmann::json;
using namespace std;

string mqtt_broker_ip; // Global variable to store the MQTT broker's IP address

const string TOPIC("robot/map");

MapManager::MapManager() {
    // Read configuration from config.json
    ifstream configFile("config.json");
    json config;
    configFile >> config;
    robot_id = config["robot_id"];
    mqtt_broker_ip = config["mqtt_broker_ip"]; // Read the MQTT broker's IP address
    configFile.close();
    cout << "MapManager: Robot ID: " << robot_id << endl;

    // Initialize the MQTT client with the correct broker IP and client ID
    client_ = new mqtt::async_client("tcp://" + mqtt_broker_ip + ":1883", "MapManagerClient_" + std::to_string(robot_id));

    for (auto &row : occupancy_grid) {
        for (auto &column : row) {
            column = -1;
        }
    }

    // Initialize the MQTT client
    initialize_mqtt_client();
    sendGridToMQTT();
}


MapManager::~MapManager() {
    client_->disconnect()->wait();
    cout << "Disconnected" << endl;
    delete client_; // Free the dynamically allocated memory
}

void MapManager::dilateCell(int x, int y, int value, double radius) {
    int grid_numbers_in_radius = (int)(radius/GRID_SIZE) + 1;
    bool updated = false;
    for (int row = y - grid_numbers_in_radius; row <= y + grid_numbers_in_radius; row++) {
        if (row < 0 || row >= MAP_SIZE) continue;
        for (int column = x - grid_numbers_in_radius; column <= x + grid_numbers_in_radius; column++) {
            if (column < 0 || column >= MAP_SIZE) continue;
            if ((abs(row-y) + abs(column-x)) <= grid_numbers_in_radius+1) {
                if (occupancy_grid[MAP_SIZE-row][column] != 1) {
                    occupancy_grid[MAP_SIZE-row][column] = value;
                    updated = true;
                }
            }
        }
    }
    if (updated && value == 1) {
        sendGridToMQTT();
    }
}

void MapManager::sendGridToMQTT() {
    auto current_time = std::chrono::steady_clock::now();
    auto time_since_last_sent = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_sent_time).count();

    if (time_since_last_sent >= 2) {  // Check if at least 1 second has passed
        // Flatten the 2D array into a 1D array
        vector<int> flattened_data;
        for (auto &row : occupancy_grid)
        {
            for (auto &column : row)
            {
                flattened_data.push_back(column);
            }
        }
        // Create a JSON object and add robot_id and grid data
        nlohmann::json j;
        j["robot_id"] = robot_id;
        j["grid"] = flattened_data;

        // Convert the JSON object to a string
        std::string payload = j.dump();

        // Publish the message with robot_id and grid data to the MQTT broker
        client_->publish(TOPIC, payload);
        std::cout << "Grid map sent to MQTT" << std::endl;        
        last_sent_time = current_time;  // Update the timestamp of the last sent time
    }
}

void MapManager::updateMap(double x, double y, int value, double radius) {
    int row = (int)round(x / GRID_SIZE) + MAP_ORIGIN;
    int column = (int)round(y / GRID_SIZE) + MAP_ORIGIN;
    //std::cout << "updateMap x:" << row << " y:" << column << " val:" << value << std::endl;
    if (row < 0 || row >= MAP_SIZE || column < 0 || column >= MAP_SIZE) {
        cout << "updateMap Coordinates out of map boundaries row" << row << " column" <<
             column << " value" << value << " radius" << radius << " x" << x << " y" << y << endl;
        throw std::out_of_range("updateMap Coordinates out of map boundaries");
    }
    dilateCell(row, column, value, radius);
    return;
}

bool MapManager::checkMap(double x, double y) {
    int row = (int)round(x / GRID_SIZE) + MAP_ORIGIN;
    int column = (int)round(y / GRID_SIZE) + MAP_ORIGIN;
    if (row < 0 || row >= MAP_SIZE || column < 0 || column >= MAP_SIZE) {
        cout << "checkMap Coordinates out of map boundaries row" << row << " column" << column << endl;
        throw std::out_of_range("checkMap Coordinates out of map boundaries");
    }
    return (occupancy_grid[MAP_SIZE-column][row] == 1);
}

// New methods for polar coordinate support
void MapManager::updateMapPolar(double distance, double angle, double initial_x, double initial_y, int value, double radius) {
    double x = distance * cos(angle) + initial_x; // Convert to Cartesian x with initial point
    double y = distance * sin(angle) + initial_y; // Convert to Cartesian y with initial point
    updateMap(x, y, value, radius); // Delegate to updateMap with Cartesian coordinates
}

bool MapManager::checkMapPolar(double distance, double angle, double initial_x, double initial_y) {
    double x = distance * cos(angle) + initial_x; // Convert to Cartesian x with initial point
    double y = distance * sin(angle) + initial_y; // Convert to Cartesian y with initial point
    return checkMap(x, y); // Delegate to checkMap with Cartesian coordinates
}

void MapManager::handleMapMessage(const std::vector<int>& received_map_data) {
    cout << "Received shared map data" << endl;
    // Deserialize the received map data into the occupancy grid format
    std::vector<int> received_grid(MAP_SIZE * MAP_SIZE, -1); // Assuming a default value of -1 for unknown cells
    for (size_t i = 0; i < received_map_data.size(); ++i) {
        received_grid[i] = received_map_data[i];
    }
    cout << "handleMapMessage map data" << endl;

    // Merge the received grid with the local occupancy grid
    mergeMap(received_grid);
}

void MapManager::mergeMap(const std::vector<int>& received_grid) {
    cout << "Merging received map with local map" << endl;
    // Assuming received_grid is a 1D vector representing the received occupancy grid
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            int index = i * MAP_SIZE + j;
            // Simple conflict resolution: if either grid has an obstacle, mark it as an obstacle
            if (received_grid[index] == 1) {
                occupancy_grid[i][j] = 1;
            } else if (received_grid[index] == 0 && occupancy_grid[i][j] == -1) {
                occupancy_grid[i][j] = 0;
            }
        }
    }
}

void MapManager::printMap(double robot_x, double robot_y) {
    int robot_column = (int)round(robot_x / GRID_SIZE) + MAP_ORIGIN;
    int robot_row = MAP_SIZE - ((int)round(robot_y / GRID_SIZE) + MAP_ORIGIN);
    cout << robot_row << robot_column << endl;
    int row_cnt = 0;
    for (auto &row : occupancy_grid) {
        bool allZeros = true; // skip the lines that contain only zeros:
        for (auto &column : row) {
            if (column != -1) {
                allZeros = false;
                break;
            }
        }
        if (!allZeros) {
            int column_cnt = 0;
            for (auto &column : row) {
                if (robot_row == row_cnt && robot_column == column_cnt) {
                    cout << "+";
                } else if (column == -1) {
                    cout << ".";
                } else if (column == 0) {
                    cout << " ";
                } else {
                    cout << "#";
                }
                column_cnt++;
            }
            cout << endl;
        }
        row_cnt++;
    }
    sendGridToMQTT();
}

void MapManager::initialize_mqtt_client() {
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    // Set the message callback
    client_->set_message_callback([this](mqtt::const_message_ptr msg) {
        if (msg->get_topic() == TOPIC) {
            std::string payload = msg->to_string();
            try {
                // Attempt to convert the string payload to a double
                auto j = json::parse(payload);
                int id = j["robot_id"];
                if (id != robot_id) {
                    vector<int> flattened_data = j["grid"];
                    // Handle the map message
                    handleMapMessage(flattened_data);
                }
            } catch (json::parse_error& e) {
                std::cerr << "Parsing error: " << e.what() << '\n';
            } catch (json::type_error& e) {
                std::cerr << "Type error: " << e.what() << '\n';
            } catch (std::exception& e) {
                std::cerr << "Some other error: " << e.what() << '\n';
            }
        }
    });

    // Connect to the MQTT broker
    try {
        cout << "MapManager: Connecting to the MQTT server..." << endl;
        client_->connect(connOpts)->wait();
        cout << "MapManager: Connected to the MQTT server" << endl;

        // Subscribe to the shared map topic
        client_->subscribe(TOPIC, 1);
    } catch (const mqtt::exception& e) {
        cerr << "MapManager: Error connecting to the MQTT server: " << e.what() << endl;
    }
}


double MapManager::calculateZeroPercentageAroundOrigin(double radius_meters) {
    int radius_cells = static_cast<int>(radius_meters / GRID_SIZE);
    
    // Calculate grid indices for the specified origin
    int origin_x = static_cast<int>(round((-0.5 / GRID_SIZE) + MAP_ORIGIN));
    int origin_y = static_cast<int>(round((0.5 / GRID_SIZE) + MAP_ORIGIN));

    int total_cells = 0;
    int zero_cells = 0;
    // Check the cells around the origin within the radius
    for (int row = origin_y - radius_cells; row < origin_y + radius_cells; ++row) {
        if (row < 0 || row >= MAP_SIZE) continue;
        for (int col = origin_x - radius_cells; col < origin_x + radius_cells; ++col) {
            if (col < 0 || col >= MAP_SIZE) continue;
            total_cells++;
            if (occupancy_grid[MAP_SIZE-row][col] == 0) {
                zero_cells++;
            }
        }
    }
    cout << "radius_cells " << radius_cells << " origin_x " << origin_x << " origin_y " << origin_y << " total_cells " << 
    total_cells << " zero_cells " << zero_cells << endl;

    double zero_percentage = 0.0;
    if (total_cells > 0) {
        zero_percentage = static_cast<double>(zero_cells) / total_cells * 100.0;
    }

    return zero_percentage;
}

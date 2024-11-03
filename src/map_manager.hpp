/**
 * @file map_manager.hpp
 *
 * @brief Simple map management module for occupancy grid algorithms
**/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "mqtt/async_client.h"

/*****************************************************************************
** Defines
*****************************************************************************/

#define ROBOT_RADIUS 0.17 // m
#define GRID_SIZE 0.05 // m

/*****************************************************************************
** Classes
*****************************************************************************/

class MapManager
{
public:
    /**
    * @brief Constructor for the MapManager class.
    *
    * The constructor initializes the occupancy grid, setting all cells to an unexplored state (-1). It prepares the map for occupancy updates and checks.
    */
    MapManager();
    /**
    * @brief Destructor for the MapManager class.
    *
    * The destructor is currently empty as there's no specific cleanup or resource release required for the MapManager.
    */
    ~MapManager();

    /**
    * @brief Updates the occupancy grid map based on the provided coordinates and value, applying dilation for obstacle representation.
    *
    * This method updates the occupancy grid map based on the provided coordinates and the corresponding value,
    * signifying the state of the area (e.g., obstacle or free space). 
    * It employs a dilation technique to represent obstacles based on a point-robot assumption,
    * where the robot is considered as a point and obstacles are dilated using the robot's radius.
    *
    * @param x The x-coordinate in meters where the update occurs.
    * @param y The y-coordinate in meters where the update occurs.
    * @param value The value indicating the state of the area (typically 0 for free space, 1 for obstacle).
    * @param radius The dilation radius around the specified coordinates due to the robot's assumed size (default: ROBOT_RADIUS).
    *
    * The update process involves converting the provided coordinates to grid indices, and then, using dilation, 
    * marks the corresponding cells in the grid based on the specified value and dilation radius.
    *
    * Note: It is important to ensure valid coordinates within the map boundaries for accurate updates.
    */
    void updateMap(double x, double y, int value, double radius = ROBOT_RADIUS);

    /**
    * @brief Checks the occupancy status in the occupancy grid map at the specified coordinates.
    *
    * This method examines the occupancy grid map at the provided coordinates to determine the occupancy status, 
    * primarily used for movement control decisions. It assesses whether the specified location contains an obstacle or is free for movement.
    *
    * @param x The x-coordinate in meters to check for occupancy.
    * @param y The y-coordinate in meters to check for occupancy.
    * @return A boolean value:
    *         - 'true' if the specified coordinates contain an obstacle.
    *         - 'false' if the specified coordinates are free for movement.
    *
    * The method converts the given coordinates to grid indices and checks the corresponding cell in the occupancy grid to ascertain the presence of an obstacle.
    *
    * Note: Ensure that the provided coordinates are within the map boundaries for accurate occupancy status checks.
    */
    bool checkMap(double x, double y);

    /**
    * @brief Updates the occupancy grid using polar coordinates and the initial point.
    *
    * This method updates the occupancy grid based on the provided polar coordinates (distance and angle), the initial point coordinates, and the corresponding value, signifying the state of the area (e.g., obstacle or free space). It employs a dilation technique to represent obstacles based on a point-robot assumption, where the robot is considered as a point and obstacles are dilated using the robot's radius.
    *
    * @param distance The distance from the origin in polar coordinates.
    * @param angle The angle in radians from the x-axis in polar coordinates.
    * @param initial_x The initial x-coordinate (in Cartesian) for the polar coordinates.
    * @param initial_y The initial y-coordinate (in Cartesian) for the polar coordinates.
    * @param value The value indicating the state of the area (typically 0 for free space, 1 for obstacle).
    * @param radius The dilation radius around the specified coordinates due to the robot's assumed size (default: ROBOT_RADIUS).
    *
    * The method calculates the Cartesian coordinates using the initial point and provided polar coordinates and then delegates to the updateMap function for Cartesian coordinates.
    */
    void updateMapPolar(double distance, double angle, double initial_x, double initial_y, int value, double radius = ROBOT_RADIUS);

    /**
    * @brief Checks the occupancy status in the occupancy grid using polar coordinates and the initial point.
    *
    * This method examines the occupancy grid at the provided polar coordinates and the initial point to determine the occupancy status, primarily used for movement control decisions. It assesses whether the specified location contains an obstacle or is free for movement.
    *
    * @param distance The distance from the origin in polar coordinates.
    * @param angle The angle in radians from the x-axis in polar coordinates.
    * @param initial_x The initial x-coordinate (in Cartesian) for the polar coordinates.
    * @param initial_y The initial y-coordinate (in Cartesian) for the polar coordinates.
    * @return A boolean value:
    *         - 'true' if the specified coordinates contain an obstacle.
    *         - 'false' if the specified coordinates are free for movement.
    *
    * The method converts the polar coordinates to Cartesian coordinates using the initial point and then delegates to the checkMap function for Cartesian coordinates.
    */
    bool checkMapPolar(double distance, double angle, double initial_x, double initial_y);

    /**
    * @brief Prints the current occupancy grid map in ASCII format.
    *
    * This method generates a visual representation of the occupancy grid map, displaying each cell's
    * occupancy status using ASCII characters. It helps visualize the map, showing unexplored areas,
    * free spaces, and occupied areas.
    *
    * The ASCII representation is as follows:
    *   - '.' represents unexplored areas (cell value = -1 in the grid).
    *   - ' ' (space) represents free space (cell value = 0 in the grid).
    *   - '#' represents occupied space (any value other than -1 or 0 in the grid).
    */
    void printMap(double robot_x, double robot_y);
    void sendGridToMQTT();
    double calculateZeroPercentageAroundOrigin(double radius_meters);

private:

    static const int MAP_SIZE = 320; // n of cells
    static const int MAP_ORIGIN = 160; // origin point
    //static const double GRID_SIZE = 0.05; // m
    mqtt::async_client* client_;
    int robot_id;
    std::chrono::steady_clock::time_point last_sent_time;  // Timestamp of the last grid sent

    /**
    * @brief 2D array representing the occupancy grid map.
    *
    * The 2D array 'occupancy_grid' serves as the representation of the occupancy grid map. It stores the occupancy status of each cell, where values:
    *   - '-1' represent unexplored areas.
    *   - '0' represent free space.
    *   - '1' or other values represent occupied areas.
    */
    int occupancy_grid[MAP_SIZE][MAP_SIZE];

    /**
    * @brief Variables for detecting obstacles in the vicinity.
    *
    * Additional boolean variables are used to track obstacles in different directions around the robot, aiding in movement control decisions.
    */
    bool left_obstacle, left_front_obstacle, front_obstacle, right_front_obstacle, right_obstacle;

    /**
    * @brief Dilates the cell in the occupancy grid based on the robot's assumed radius.
    *
    * This method dilates the cell in the occupancy grid at the specified indices (x, y) based on the provided value and the given radius. 
    * The dilation simulates the expansion of obstacles around the specified point, representing the robot's physical size in the occupancy map.
    *
    * @param x The x-index of the cell in the occupancy grid.
    * @param y The y-index of the cell in the occupancy grid.
    * @param value The value to set for the dilated cells (typically 0 for free space, 1 for obstacle).
    * @param radius The dilation radius around the specified coordinates due to the robot's assumed size (default: ROBOT_RADIUS).
    */
    void dilateCell(int x, int y, int value, double radius = ROBOT_RADIUS);

    void mergeMap(const std::vector<int>& received_grid);

    // Function to initialize the MQTT client
    void initialize_mqtt_client();

    // Callback function for handling received map messages
    void handleMapMessage(const std::vector<int>& received_map_data);
};

#endif /* MAP_MANAGER_HPP_ */
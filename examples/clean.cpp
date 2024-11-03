#include <string>
#include <csignal>
#include <fstream>
#include <vector>
#include <sstream>
#include <chrono>
#include <ctime>
#include <ecl/geometry.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/command_line.hpp>
#include "kobuki_core/kobuki.hpp"
#include "kobuki_core/packets/core_sensors.hpp"

#define FORWARD_SPEED 0.25
#define ROTATION_SPEED 1.0
#define FAST_ROTATION_SPEED 1.3
#define MAP_SIZE 200 // n of cells
#define MAP_ORIGIN 100 // origin point is at [100][100]
#define GRID_SIZE 0.05 // m
#define ROBOT_RADIUS 0.17 // m

#define LEFT_BUMPER 4
#define RIGHT_BUMPER 1
#define CENTER_BUMPER 2

using namespace std;
/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager
{
public:
    KobukiManager(
        const std::string &device,
        const double &length,
        const bool &disable_smoothing,
        const double target_x,
        const double target_y) : dx(0.0), dth(0.0),
                                 length(length),
                                 slot_stream_data(&KobukiManager::processStreamData, *this)
    {
        this->temp_target_x = this->target_x = target_x;
        this->temp_target_y = this->target_y = target_y;
        kobuki::Parameters parameters;
        parameters.sigslots_namespace = "/kobuki";
        parameters.device_port = device;
        parameters.enable_acceleration_limiter = !disable_smoothing;
        robot_mode = GO_TO_GOAL_MODE;
        moving_state = ADJUST_HEADING;
        for (auto &row : occupancy_grid)
        {
            for (auto &column : row)
            {
                column = -1;
            }
        }
        kobuki.init(parameters);
        kobuki.enable();
        slot_stream_data.connect("/kobuki/stream_data");
    }

    ~KobukiManager()
    {
        kobuki.setBaseControl(0, 0); // linear_velocity, angular_velocity in (m/s), (rad/s)
        kobuki.disable();
    }

    void processStreamData()
    {
        ecl::linear_algebra::Vector3d pose_update;
        ecl::linear_algebra::Vector3d pose_update_rates;
        kobuki.updateOdometry(pose_update, pose_update_rates);
        pose = ecl::concatenate_poses(pose, pose_update);
        current_x = pose[0];
        current_y = pose[1];
        pose[2] = current_yaw = kobuki.getHeading(); // override broken odometry heading with the gyro data
        dx += pose_update[0];  // dx
        dth += pose_update[2]; // dheading
        data = kobuki.getCoreSensorData();
        processMotion();
        
    }

    void dilateCell(int x, int y, int value) {
        int grid_numbers_in_radius = (int)(ROBOT_RADIUS/GRID_SIZE)+1;
        for (int row = y - grid_numbers_in_radius; row <= y + grid_numbers_in_radius; row++) {
            if (row < 0 || row >= MAP_SIZE) continue;
            for (int column = x - grid_numbers_in_radius; column <= x + grid_numbers_in_radius; column++) {
                if (column < 0 || column >= MAP_SIZE) continue;
                if ((abs(row-y) + abs(column-x)) <= grid_numbers_in_radius+1) {
                    if (occupancy_grid[MAP_SIZE-row][column] != 1) {
                        occupancy_grid[MAP_SIZE-row][column] = value;
                    }
                }
            }
        }
    }

    // Technique used for mapping is Point-robot assumption. 
    //Meaning that the robot is represented as a point, and obstacles are Dilated by robot’s radius
    void updateMap()
    {
        // Mark the object cell from the hit point, and use Dilation function
        int x, y;
        if (data.bumper == LEFT_BUMPER) { // if left bumper is hit,
            x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw + ecl::pi * 0.25)) / GRID_SIZE) + MAP_ORIGIN;
            y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw + ecl::pi * 0.25)) / GRID_SIZE) + MAP_ORIGIN;
            cout << "left hit ["  << x << " " << y << endl;
            dilateCell(x, y, 1);
        } else if (data.bumper == RIGHT_BUMPER) { // if right bumper is hit
            x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw - ecl::pi * 0.25)) / GRID_SIZE) + MAP_ORIGIN;
            y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw - ecl::pi * 0.25)) / GRID_SIZE) + MAP_ORIGIN;
            cout << "right hit ["  << x << " " << y << endl;
            dilateCell(x, y, 1);
        } else if ((int)data.bumper != 0) { // if any other bumper combination is hit, mark the front point
            x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw)) / GRID_SIZE) + MAP_ORIGIN;
            y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw)) / GRID_SIZE) + MAP_ORIGIN;
            cout << " hit ["  << x << " " << y << endl;
            dilateCell(x, y, 1);
        } else { // if no hit, mark the current cell as clean. Note that if the cell is already 1, it cannot be overwritten
            x = (int)round(current_x / GRID_SIZE) + MAP_ORIGIN;
            y = (int)round(current_y / GRID_SIZE) + MAP_ORIGIN;
            dilateCell(x, y, 0);
        }
        return;
    }

    void checkMap() // check obstacles in front for movement controls
    {
        front_obstacle = right_front_obstacle = right_obstacle = left_front_obstacle = left_obstacle = false;
        int x, y; // row, column
        // check front
        x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw)) / GRID_SIZE) + MAP_ORIGIN;
        y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw)) / GRID_SIZE) + MAP_ORIGIN;
        if (occupancy_grid[MAP_SIZE-y][x] == 1) front_obstacle = true;
        // check right_front
        x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw-ecl::pi/6)) / GRID_SIZE) + MAP_ORIGIN;
        y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw-ecl::pi/6)) / GRID_SIZE) + MAP_ORIGIN;
        if (occupancy_grid[MAP_SIZE-y][x] == 1) right_front_obstacle = true;
        // check right
        x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw-ecl::pi*0.5)) / GRID_SIZE) + MAP_ORIGIN;
        y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw-ecl::pi*0.5)) / GRID_SIZE) + MAP_ORIGIN;
        if (occupancy_grid[MAP_SIZE-y][x] == 1) right_obstacle = true;
        // check left_front
        x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw+ecl::pi/6)) / GRID_SIZE) + MAP_ORIGIN;
        y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw+ecl::pi/6)) / GRID_SIZE) + MAP_ORIGIN;
        if (occupancy_grid[MAP_SIZE-y][x] == 1) left_front_obstacle = true;
        // check left
        x = (int)round((current_x + ROBOT_RADIUS * cos(current_yaw+ecl::pi*0.5)) / GRID_SIZE) + MAP_ORIGIN;
        y = (int)round((current_y + ROBOT_RADIUS * sin(current_yaw+ecl::pi*0.5)) / GRID_SIZE) + MAP_ORIGIN;
        if (occupancy_grid[MAP_SIZE-y][x] == 1) left_obstacle = true;
    }

    void print(bool map)
    {
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);
        cout << endl << "Pose: [" << current_x << ", " << current_y << ", " << current_yaw*360.0/ecl::pi 
                << "] " << std::put_time(std::localtime(&now_time), "%F %T") << std::endl;
        cout << "Obstacles L LF F RF R: " << left_obstacle << left_front_obstacle  
                << front_obstacle << right_front_obstacle << right_obstacle << endl;

        if (!map) return;
        for (auto &row : occupancy_grid)
        {
            for (auto &column : row)
            {     
                if (column != -1) cout << column;
                else cout << " "; // print undiscovered cells as empty
            }
            cout << endl;
        }
    }

    double getYawError()
    {
        // Calculate the desired heading based on the current position
        // and the desired position
        double desired_yaw = atan2(
            target_y - current_y,
            target_x - current_x);
        // How far off is the current heading in radians?
        double yaw_error = desired_yaw - current_yaw;
        return ecl::wrap_angle(yaw_error);
    }

    // Generate motion
    void processMotion()
    {
        const double buffer = 0.05;
        double longitudinal_velocity = 0.0;
        double rotational_velocity = 0.0;
        checkMap();

        if (robot_mode == GO_TO_GOAL_MODE)
        { // "go to target mode"
            if (moving_state == GO_STRAIGHT && (data.bumper != 0 || right_front_obstacle || front_obstacle || left_front_obstacle))
            {   // HIT!
                kobuki.setBaseControl(longitudinal_velocity, rotational_velocity);
                robot_mode = WALL_FOLLOWING_MODE; // wall following mode
                // save hit point coordinates:
                hit_x = current_x;
                hit_y = current_y;
                distance_to_goal_from_hit_point = sqrt((
                        pow(target_x - hit_x, 2)) +
                        (pow(target_y - hit_y, 2)));
                cout << "m0, hit_x: " << hit_x << " hit_y: " << hit_y << " distance_to_goal_from_hit_point: " 
                        << distance_to_goal_from_hit_point << endl;
                std::cout << "robot_mode: " << robot_mode << " WALL_FOLLOWING_MODE, moving_mode: " << moving_state << endl;
                // stop and switch to wall mode
                return;
            }

            if (moving_state == ADJUST_HEADING)
            { // ADJUST HEADING
                double yaw_error = getYawError();
                // Adjust heading if heading is not good enough
                if (fabs(yaw_error) > yaw_precision)
                {
                    if (yaw_error > 0.5)
                    {
                        // Turn left (counterclockwise)
                        rotational_velocity = FAST_ROTATION_SPEED;
                    }
                    else if (yaw_error > 0)
                    { // very close, rotate slowly
                        // Turn left (counterclockwise)
                        longitudinal_velocity = FORWARD_SPEED * 0.5;
                        rotational_velocity = ROTATION_SPEED;
                    }
                    else if (yaw_error < -0.5)
                    {
                        // Turn right (clockwise)
                        rotational_velocity = -FAST_ROTATION_SPEED;
                    }
                    else
                    { // very close, rotate slowly
                        // Turn right (clockwise)
                        longitudinal_velocity = FORWARD_SPEED * 0.5;
                        rotational_velocity = -ROTATION_SPEED;
                    }
                }
                else
                {
                    moving_state = GO_STRAIGHT;
                    std::cout << "robot_mode: " << robot_mode << ", moving_mode: GO_STRAIGHT " << moving_state << endl;
                }
            }
            else if (moving_state == GO_STRAIGHT)
            { // GO STRAIGHT
                double position_error = sqrt(
                    pow(target_x - current_x, 2) + pow(target_y - current_y, 2));

                if (position_error > 0.2)
                {
                    longitudinal_velocity = FORWARD_SPEED;
                    // How far off is the current heading in radians?
                    double yaw_error = getYawError();

                    // Adjust heading if heading is not good enough
                    if (fabs(yaw_error) > yaw_precision + 0.2)
                    {
                        moving_state = ADJUST_HEADING; // ADJUST HEADING
                        std::cout << "robot_mode: " << robot_mode << ", moving_mode: " << moving_state << endl;
                    }
                }
                else
                {   // If distance to target is smaller than 20cm
                    moving_state = GOAL_ACHIEVED; // finish successfully
                    std::cout << "robot_mode: " << robot_mode << ", moving_mode: " << moving_state << endl;
                    kobuki.setBaseControl(0.0, 0.0);
                    cout << "DONE!" << endl;
                }
            }
            else if (moving_state == GOAL_ACHIEVED)
            { // GOAL_ACHIEVED
                if (data.buttons & kobuki::CoreSensors::Flags::Button0)
                {
                    cout << "B0 pressed!!!" << endl;
                    if (fabs(target_x) > 0.1)
                    {
                        target_x = 0.0;
                    }
                    else
                    {
                        target_x = temp_target_x;
                    }
                    target_y = 0.0;
                    moving_state = ADJUST_HEADING;
                    std::cout << "robot_mode: " << robot_mode << ", moving_mode: " << moving_state << endl;
                }
            }
        }
        else if (robot_mode == WALL_FOLLOWING_MODE)
        { // "wall following mode"
            // Distance to the line:
            double a = hit_y - target_y;
            double b = target_x - hit_x;
            double c = hit_x * target_y - target_x * hit_y;
            double distance_to_target_line = abs(a * current_x + b * current_y + c) / sqrt(a * a + b * b);
            if (distance_to_target_line < 0.10)
            { // If we hit the start-goal line again?
                // Is the leave point closer to the goal than the hit point?
                // If yes, go to goal. (Should be at least 20cm closer in order to avoid looping)
                double distance_to_goal_from_crossing_point = sqrt(
                    pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
                if ((distance_to_goal_from_hit_point - distance_to_goal_from_crossing_point) > 0.2)
                {
                    cout << "HIT the GOAL LINE! ";
                    robot_mode = GO_TO_GOAL_MODE; // "go to goal mode"
                    moving_state = ADJUST_HEADING;
                    std::cout << "robot_mode: " << robot_mode << ", moving_mode: " << moving_state << endl;
                    kobuki.setBaseControl(longitudinal_velocity, rotational_velocity);
                    return;
                }
            }

            // BUMPERS: 0, 1=R, 2=C, 4=L, 3=RC, 5=RL, 6=CL, 7=RCL
            // if hit
            if (data.bumper != 0)
            {
                // move backwards
                longitudinal_velocity = -FORWARD_SPEED * 0.5;
                dx = 0.0;
                dth = 0.0;
                corners_turned = 0;
            }
            else if (front_obstacle || left_front_obstacle)
            {
                // turn left and move forward slowly
                rotational_velocity = FAST_ROTATION_SPEED;
                dx = 0.0;
                dth = 0.0;
                corners_turned = 0;
            }
            else if (right_front_obstacle)
            {
                // turn left and move forward slowly
                longitudinal_velocity = FORWARD_SPEED * 0.5;
                rotational_velocity = FAST_ROTATION_SPEED;
                dx = 0.0;
                dth = 0.0;
                corners_turned = 0;
            }
            else if (right_obstacle)
            {
                // move straight to follow the wall
                longitudinal_velocity = FORWARD_SPEED;
                dx = 0.0;
                dth = 0.0;
                corners_turned = 0;
            } else if (data.bumper == 0)
            {
                // turn right and move forward slowly
                longitudinal_velocity = FORWARD_SPEED;
                rotational_velocity = -FAST_ROTATION_SPEED;
            }
            else
            {
                dx = 0.0;
                dth = 0.0;
                corners_turned = 0;
            }
        }
        kobuki.setBaseControl(longitudinal_velocity, rotational_velocity);
    }

private:
    double dx, dth;
    int corners_turned = 0;
    const double length;
    ecl::linear_algebra::Vector3d pose;      // x, y, heading
    ecl::linear_algebra::Vector3d test_pose; // x, y, heading
    kobuki::Kobuki kobuki;
    ecl::Slot<> slot_stream_data;
    kobuki::CoreSensors::Data data;
    // Current position and orientation of the robot
    double current_x;
    double current_y;
    double current_yaw;
    // Target coordinates from a file
    double target_x;
    double target_y;
    double temp_target_x;
    double temp_target_y;
    // Coordinates of the first hit point
    double hit_x;
    double hit_y;
    double distance_to_goal_from_hit_point;
    // +/- 5.0 degrees of precision for the rotation angle
    double yaw_precision = 5.0 * (ecl::pi / 180);
    int occupancy_grid[MAP_SIZE][MAP_SIZE];
    // variables for occupancy grid obstacles closer than 10cm
    bool left_obstacle, left_front_obstacle, front_obstacle, right_front_obstacle, right_obstacle;
    /*  ############# MAIN ROBOT MODES ###################
        "go to goal mode": Robot will head to an x,y coordinate
        "wall following mode": Robot will follow a wall */
    enum robot_mode_enum
    {
        GO_TO_GOAL_MODE,
        WALL_FOLLOWING_MODE
    } robot_mode;

    /*  ############ GO TO TARGET MODES ##################
        adjust heading: Rotate towards the target
        go straight: Go straight towards the target
        goal achieved: Reached the goal x, y coordinates */
    enum moving_state_enum
    {
        ADJUST_HEADING,
        GO_STRAIGHT,
        GOAL_ACHIEVED
    } moving_state;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool shutdown_req = false;
void signalHandler(int /* signum */)
{
    shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
    fstream fs;
    fs.open("target.txt", ios::in);
    vector<vector<float>> floatVec;
    string strFloat;
    float targetX;
    float targetY;
    int counter = 0;
    getline(fs, strFloat);
    cout << fixed;
    cout.precision(3);
    std::stringstream linestream(strFloat);
    linestream >> targetX;
    linestream >> targetY;
    std::cout << "x: " << targetX << " y: " << targetY << std::endl;

    ecl::ValueArg<std::string> device_port(
        "p", "port",
        "Path to device file of serial port to open",
        false,
        "/dev/kobuki",
        "string");
    ecl::ValueArg<double> length(
        "l", "length",
        "traverse square with sides of this size in length (m)",
        false,
        0.15,
        "double");
    ecl::SwitchArg disable_smoothing(
        "d", "disable_smoothing",
        "Disable the acceleration limiter (smoothens velocity)",
        false);

    cmd_line.add(device_port);
    cmd_line.add(length);
    cmd_line.add(disable_smoothing);
    cmd_line.parse(argc, argv);

    signal(SIGINT, signalHandler);

    std::cout << "Demo : Example of simple control loop. l: " << length.getValue() << std::endl;
    KobukiManager kobuki_manager(
        device_port.getValue(),
        length.getValue(),
        disable_smoothing.getValue(),
        targetX,
        targetY);

    ecl::MilliSleep sleep(1);
    ecl::linear_algebra::Vector3d pose; // x, y, heading
    try
    {
        while (!shutdown_req)
        {
            sleep(200);
            kobuki_manager.print(false);
            kobuki_manager.updateMap();
        }
    }
    catch (ecl::StandardException &e)
    {
        std::cout << e.what();
    }
    sleep(300);
    kobuki_manager.print(true);

    return 0;
}
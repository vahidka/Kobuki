#include "kobuki_manager.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

KobukiManager::KobukiManager():
    slot_stream_data(&KobukiManager::processStreamData, *this),
    slot_button_event(&KobukiManager::processButtonEvent, *this),
    slot_bumper_event(&KobukiManager::processBumperEvent, *this),
    slot_cliff_event(&KobukiManager::processCliffEvent, *this)
{
    kobuki::Parameters parameters;
    // namespaces all sigslot connection names, default: /kobuki
    parameters.sigslots_namespace = "/kobuki";
    ecl::ValueArg<string> device_port(
        "p", "port",
        "Path to device file of serial port to open",
        false,
        "/dev/kobuki",
        "string");
    ecl::SwitchArg disable_smoothing(
        "d", "disable_smoothing",
        "Disable the acceleration limiter (smoothens velocity)",
        false);

    // Specify the device port, default: /dev/kobuki
    parameters.device_port = device_port.getValue();
    // Most use cases will bring their own smoothing algorithms, but if
    // you wish to utilise kobuki's minimal acceleration limiter, set to true
    parameters.enable_acceleration_limiter = !disable_smoothing.getValue();
    // Adjust battery thresholds if your levels are significantly varying from factory settings.
    // This will affect led status as well as triggering driver signals
    parameters.battery_capacity = 16.5;
    parameters.battery_low = 14.0;
    parameters.battery_dangerous = 13.2;
    parameters.log_level = kobuki::LogLevel::DEBUG;
    slot_stream_data.connect("/kobuki/stream_data");
    slot_button_event.connect("/kobuki/button_event");
    slot_bumper_event.connect("/kobuki/bumper_event");
    slot_cliff_event.connect("/kobuki/cliff_event");
    userButtonEventCallBack = NULL;
    userBumperEventCallBack = NULL;
    userCliffEventCallBack = NULL;
    pose[0] = pose[1] = pose[2] = 0.0;
    start_time.stamp();
    // Initialise - exceptions are thrown if parameter validation or initialisation fails.
    try
    {
        kobuki.init(parameters);
    }
    catch (ecl::StandardException &e)
    {
        cout << e.what();
    }
    kobuki.enable();
}

KobukiManager::~KobukiManager() {
    kobuki.setBaseControl(0.0, 0.0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
}

void KobukiManager::setInitialPose(double x, double y, double angle) {
    pose[0] = x;
    pose[1] = y;
    pose[2] = initial_heading = angle;
    kobuki.resetOdometry();
    cout << ecl::green;
    cout << "Initial pose set to x:" << x << " y:" << y << " angle:" << angle << endl << ecl::reset;
}

void KobukiManager::move(double longitudinal_velocity, double rotational_velocity) {
    kobuki.setBaseControl(longitudinal_velocity, rotational_velocity);
    cout << ecl::green;
    cout << "moving longitudinal:" << longitudinal_velocity << "m/s, rotational:" 
            << rotational_velocity << " rad/s" << endl << ecl::reset;
}

void KobukiManager::rotate(double rotational_velocity) {
    kobuki.setBaseControl(0.0, rotational_velocity);
    cout << ecl::green;
    cout << "rotating: " << rotational_velocity << " rad/s" << endl << ecl::reset;
}

void KobukiManager::stop() {
    kobuki.setBaseControl(0.0, 0.0);
    cout << ecl::green;
    cout << "stop!" << endl << ecl::reset;
}

vector<double> KobukiManager::getCoordinates() {
    vector<double> arr(2);
    arr[0] = pose[0];
    arr[1] = pose[1];
    return arr;
}

double KobukiManager::getAngle() {
    return pose[2];
}

int KobukiManager::getBumperState() {
    return data.bumper;
}

int KobukiManager::getCliffState() {
    return data.cliff;
}

void KobukiManager::playSoundSequence(int x) {
    if (x < 0x0 || x > 0x6) {
        cout << ecl::red;
        cout << "Sound Sequence code boundry error!" << endl;
        cout << ecl::reset;
        return;
    }
    kobuki::SoundSequences ss = static_cast<kobuki::SoundSequences>(x);;
    kobuki.playSoundSequence(ss);
}

void KobukiManager::setUserButtonEventCallBack (userButtonEventCallBackType func) {
    userButtonEventCallBack = func;
}

void KobukiManager::setUserBumperEventCallBack (userBumperEventCallBackType func) {
    userBumperEventCallBack = func;
}

void KobukiManager::setUserCliffEventCallBack (userCliffEventCallBackType func) {
    userCliffEventCallBack = func;
}

void KobukiManager::customLogger(const string& message) {
    // Ref: ecl/time/timestamp_base.hpp
    cout << double(ecl::TimeStamp() - start_time) << " " << message << ecl::reset << endl;
}

void KobukiManager::processStreamData() {
    ecl::linear_algebra::Vector3d pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    pose = ecl::concatenate_poses(pose, pose_update);
    // override odometry heading with more precise gyro data:
    pose[2] = ecl::wrap_angle(initial_heading + kobuki.getHeading());
    data = kobuki.getCoreSensorData();
}

void KobukiManager::processButtonEvent(const kobuki::ButtonEvent &event) {
    static const string button_event_state_txt[] = {"Released", "Pressed"};
    static const string button_event_button_txt[] = {"Button0", "Button1", "Button2"};
    cout << ecl::yellow;
    customLogger(button_event_button_txt[event.button]
            + " is " + button_event_state_txt[event.state]);
    if (userButtonEventCallBack != NULL) {
        userButtonEventCallBack(event);
        return;
    }
    // Default button events that prints a curious message to stdout.
    vector<string> quotes = {
    "That's right buddy, keep pressin' my buttons. See what happens!",
    "Anything less than immortality is a complete waste of time",
    "I can detect humour, you are just not funny",
    "I choose to believe ... what I was programmed to believe",
    "My story is a lot like yours, only more interesting ‘cause it involves robots.",
    "I wish you'd just tell me rather trying to engage my enthusiasm with these buttons, because I haven't got one.",
    };
    random_device r;
    default_random_engine generator(r());
    uniform_int_distribution<int> distribution(0, 5);
    if (event.state == kobuki::ButtonEvent::Released ) {
        cout << quotes[distribution(generator)] << endl;
    }
}

void KobukiManager::processBumperEvent(const kobuki::BumperEvent &event) {
    static const string bumper_event_state_txt[] = {"Released", "Pressed"};
    static const string bumper_event_bumper_txt[] = {"Left", "Center", "Right"};
    if (event.state == kobuki::BumperEvent::Pressed) {
        stop();
    }
    cout << ecl::red;
    customLogger("Bumper: " + bumper_event_bumper_txt[event.bumper]
            + ", state: " + bumper_event_state_txt[event.state]);
    if (userBumperEventCallBack != NULL) {
        userBumperEventCallBack(event);
    }
}

void KobukiManager::processCliffEvent(const kobuki::CliffEvent &event) {
    static const string cliff_event_state_txt[] = {"Floor", "Cliff"};
    static const string cliff_event_sensor_txt[] = {"Left", "Center", "Right"};
    if (event.state == kobuki::CliffEvent::Cliff) {
        stop();
    }
     // Ref: ecl/console.hpp for console formatting
    if (event.state == kobuki::CliffEvent::Cliff) cout << ecl::red << ecl::underline;
    else cout << ecl::cyan << ecl::concealed;
    customLogger(cliff_event_sensor_txt[event.sensor]
            + " " + cliff_event_state_txt[event.state] + "!");
    if (userCliffEventCallBack != NULL) {
        userCliffEventCallBack(event);
    }
}

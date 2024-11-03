#include <iostream>
#include <string>

#include <ecl/command_line.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>

#include <kobuki_core/kobuki.hpp>

typedef void (*userButtonCallBackType)(const kobuki::ButtonEvent &event);

class KobukiManager
{
public:
  KobukiManager(const std::string &device) :
      dx(0.0), dth(0.0),
      slot_stream_data(&KobukiManager::processStreamData, *this),
      slot_button_event(&KobukiManager::processButtonEvent, *this),
      slot_bumper_event(&KobukiManager::processBumperEvent, *this),
      slot_cliff_event(&KobukiManager::processCliffEvent, *this)
  {
    kobuki::Parameters parameters;
    parameters.device_port = device;
    //slot_button_event = KobukiManager::processButtonEvent;
    try
    {
      kobuki.init(parameters);
    }
    catch (ecl::StandardException &e)
    {
      std::cout << e.what();
    }
    slot_stream_data.connect("/kobuki/stream_data");
    slot_button_event.connect("/kobuki/button_event");
    slot_bumper_event.connect("/kobuki/bumper_event");
    slot_cliff_event.connect("/kobuki/cliff_event");
    userButtonCallBack = NULL;
  }

  /*
   * Nothing to do in the main thread, just put it to sleep
   */
  void move(double forward_speed)
  {
    std::cout << "move " << forward_speed << std::endl;
    kobuki.setBaseControl(forward_speed, 0.0);
  }

  void rotate(double rotation)
  {
    std::cout << "rotate " << rotation << std::endl;
    kobuki.setBaseControl(0.0, rotation);
  }

  void stop()
  {
    std::cout << "stop!" << std::endl;
    kobuki.setBaseControl(0.0, 0.0);
  }

  /*
   * Called whenever the kobuki receives a data packet.
   * Up to you from here to process it.
   */
  void processStreamData()
  {
    ecl::linear_algebra::Vector3d pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    pose = ecl::concatenate_poses(pose, pose_update);
    current_x = pose[0];
    current_y = pose[1];
    pose[2] = current_yaw = kobuki.getHeading();
    // std::chrono::duration<double> elapsed_seconds = end-start;

    dx += pose_update[0];  // dx
    dth += pose_update[2]; // dheading

    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    std::cout << "Encoders [" << data.left_encoder << "," << data.right_encoder << "]" <<
    " current_yaw: " << current_yaw << " current_x: " << current_x << " current_y: " << current_y << std::endl;
  }

  void setUserButtonCallBack (userButtonCallBackType func) {
    userButtonCallBack = func;
  }

    /*
   * Catches button events and prints a curious message to stdout.
   */
  void processButtonEvent(const kobuki::ButtonEvent &event)
  {

    if(userButtonCallBack!=NULL) {
        userButtonCallBack(event);
        return;
       }

      std::vector<std::string> quotes = {
      "That's right buddy, keep pressin' my buttons. See what happens!",
      "Anything less than immortality is a complete waste of time",
      "I can detect humour, you are just not funny",
      "I choose to believe ... what I was programmed to believe",
      "My story is a lot like yours, only more interesting ‘cause it involves robots.",
      "I wish you'd just tell me rather trying to engage my enthusiasm with these buttons, because I haven't got one.",
    };
    std::random_device r;
    std::default_random_engine generator(r());
    std::uniform_int_distribution<int> distribution(0, 5);
    if (event.state == kobuki::ButtonEvent::Released ) {
      std::cout << quotes[distribution(generator)] << std::endl;
    }
  }

      /*
   * Catches bumper events and prints a message to stdout.
   */
  void processBumperEvent(const kobuki::BumperEvent &event)
  {
    std::cout << "processBumperEvent bumper: " << event.bumper << ", state: " << event.state << std::endl;
  }
      /*
   * Catches cliff events and prints a message to stdout.
   */
  void processCliffEvent(const kobuki::CliffEvent &event)
  {
    std::cout << "processCliffEvent" << std::endl;
  }

private:
  kobuki::Kobuki kobuki;
  double dx, dth;
  ecl::linear_algebra::Vector3d pose;      // x, y, heading
  // Current position and orientation of the robot
  double current_x;
  double current_y;
  double current_yaw;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const kobuki::ButtonEvent&> slot_button_event;
  ecl::Slot<const kobuki::BumperEvent&> slot_bumper_event;
  ecl::Slot<const kobuki::CliffEvent&> slot_cliff_event;

  // Function pointer callbacks:
  userButtonCallBackType userButtonCallBack;

};

void examplePrint(const kobuki::ButtonEvent &event) {
  std::cout << "HELLO" << std::endl;
}

int main(int argc, char **argv)
{
  ecl::CmdLine cmd_line("buttons", ' ', "0.1");
  ecl::ValueArg<std::string> device_port(
      "p", "port",
      "Path to device file of serial port to open",
      false,
      "/dev/kobuki",
      "string"
  );
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);


  KobukiManager kobuki_manager(device_port.getValue());
  ecl::Sleep sleep(1);
  kobuki_manager.setUserButtonCallBack(examplePrint);
  while (1) { // main controller
    kobuki_manager.move(0.1);
    kobuki_manager.rotate(0.3);
    kobuki_manager.stop();
    sleep(1);
  }

  return 0;
}
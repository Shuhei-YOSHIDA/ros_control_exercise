/**
 * @file myrobot_controller_manager_node.cpp
 */

#include <controller_manager/controller_manager.h>
#include "ros_control_exercise/myrobot_controller.h"

int main(int argc, char** argv)
{
  MyRobotController controller;
  controller_manager::ControllerManager cm(&controller);

  while(true)
  {
    //controller.read();
    //cm.update(controller.get_time(), robot.get_period());
    //controller.write();
    //sleep();
  }

  return 0;
}

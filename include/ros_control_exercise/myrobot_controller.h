/**
 * @file myrobot_controller.h
 */

#ifndef INCLUDE_ROS_CONTROL_EXERCISE_MYROBOT_CONTROLLER_H
#define INCLUDE_ROS_CONTROL_EXERCISE_MYROBOT_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobotController : public hardware_interface::RobotHW
{
public:
  MyRobotController();

private:


};


#endif /* ifndef INCLUDE_ROS_CONTROL_EXERCISE_MYROBOT_CONTROLLER_H */

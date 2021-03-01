/**
 * @file hector_quadrotor_gimbal_hw_sim.h
 */

#ifndef INCLUDE_ROS_CONTROL_EXERCISE_HECTOR_QUADROTOR_GIMBAL_HW_SIM_H
#define INCLUDE_ROS_CONTROL_EXERCISE_HECTOR_QUADROTOR_GIMBAL_HW_SIM_H

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

// For hector_quadrotor_controller_gazebo
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/limiters.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/EnableMotors.h>

namespace ros_control_exercise
{

class DefaultRobotHwSim;
class QuadrotorHardwareSim;

class HectorQuadrotorGimbalHwSim : public gazebo_ros_control::RobotHWSim
{
public:
  HectorQuadrotorGimbalHwSim();
  virtual ~HectorQuadrotorGimbalHwSim();

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

private:
  std::shared_ptr<DefaultRobotHwSim> default_robot_sim_ptr_;
  std::shared_ptr<QuadrotorHardwareSim> quadrotor_hardware_sim_ptr_;
};


class DefaultRobotHwSim// : public gazebo_ros_control::RobotHWSim
{
public:
  DefaultRobotHwSim(HectorQuadrotorGimbalHwSim *super_hw);
  virtual ~DefaultRobotHwSim();
  HectorQuadrotorGimbalHwSim* super_hw_;

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
};

using namespace hector_quadrotor_interface;

class QuadrotorHardwareSim // : public gazebo_ros_control::RobotHWSim
{
public:
  QuadrotorHardwareSim(HectorQuadrotorGimbalHwSim *super_hw);
  virtual ~QuadrotorHardwareSim();
  HectorQuadrotorGimbalHwSim* super_hw_;

  virtual bool initSim(
      const std::string &robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  bool enableMotorsCallback(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res);

private:
  bool enableMotors(bool enable);

  std_msgs::Header header_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Accel acceleration_;
  sensor_msgs::Imu imu_;
  hector_uav_msgs::MotorStatus motor_status_;

  QuadrotorInterface interface_;

  WrenchCommandHandlePtr wrench_output_;
  MotorCommandHandlePtr motor_output_;

  hector_quadrotor_interface::WrenchLimiter wrench_limiter_;
  std::string base_link_frame_, world_frame_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::PhysicsEnginePtr physics_;

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d gz_pose_;
  ignition::math::Vector3d gz_velocity_, gz_acceleration_, gz_angular_velocity_, gz_angular_acceleration_;
#else
  gazebo::math::Pose gz_pose_;
  gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_, gz_angular_acceleration_;
#endif

  boost::shared_ptr<hector_quadrotor_interface::ImuSubscriberHelper> imu_sub_helper_;
  boost::shared_ptr<hector_quadrotor_interface::OdomSubscriberHelper> odom_sub_helper_;

  ros::Publisher wrench_command_publisher_;
  ros::Publisher motor_command_publisher_;
  ros::ServiceServer enable_motors_server_;
};

typedef boost::shared_ptr<HectorQuadrotorGimbalHwSim> HectorQuadrotorGimbalHwSimPtr;
}

#endif /* INCLUDE_ROS_CONTROL_EXERCISE_HECTOR_HECTOR_QUADROTOR_GIMBAL_HW_SIM_H */

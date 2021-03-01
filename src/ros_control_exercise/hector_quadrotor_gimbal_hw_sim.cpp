/**
 * @file hector_quadrotor_gimbal_hw_sim.cpp
 */

#include "ros_control_exercise/hector_quadrotor_gimbal_hw_sim.h"
#include <urdf/model.h>


namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace ros_control_exercise
{

HectorQuadrotorGimbalHwSim::HectorQuadrotorGimbalHwSim()
{
  default_robot_sim_ptr_ = std::make_shared<DefaultRobotHwSim>(this);
  quadrotor_hardware_sim_ptr_ = std::make_shared<QuadrotorHardwareSim>(this);
}

bool HectorQuadrotorGimbalHwSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool is_default = default_robot_sim_ptr_->initSim(
    robot_namespace,
    model_nh,
    parent_model,
    urdf_model,
    transmissions);

  bool is_quadrotor = quadrotor_hardware_sim_ptr_->initSim(
    robot_namespace,
    model_nh,
    parent_model,
    urdf_model,
    transmissions);

  return is_default && is_quadrotor;
}

void HectorQuadrotorGimbalHwSim::readSim(ros::Time time, ros::Duration period)
{
  default_robot_sim_ptr_->readSim(time, period);
  quadrotor_hardware_sim_ptr_->readSim(time, period);

  return;
}

void HectorQuadrotorGimbalHwSim::writeSim(ros::Time time, ros::Duration period)
{
  default_robot_sim_ptr_->writeSim(time, period);
  quadrotor_hardware_sim_ptr_->writeSim(time, period);

  return;
}

void HectorQuadrotorGimbalHwSim::eStopActive(const bool active)
{
  default_robot_sim_ptr_->eStopActive(active);

  return;
}


////////////////////////////////

DefaultRobotHwSim::DefaultRobotHwSim(HectorQuadrotorGimbalHwSim* super_hw)
{
  super_hw_ = super_hw;
}

bool DefaultRobotHwSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. Using the first entry");
      //continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string& hardware_interface = joint_interfaces.front();

    // Debug
    ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
    {
      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
        << hardware_interface << "' while loading interfaces for " << joint_names_[j] );
      return false;
    }

    if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
      ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_names_[j] << "'.");
    }

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM_NAMED("default_robot_hw", "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    // get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
    physics_type_ = physics->GetType();
    if (physics_type_.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "No physics type found.");
    }

    registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);
    if (joint_control_methods_[j] != EFFORT)
    {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetParam("vel") to control the joint.
      const ros::NodeHandle nh(robot_namespace + "/gazebo_ros_control/pid_gains/" +
                               joint_names_[j]);
      if (pid_controllers_[j].init(nh))
      {
        switch (joint_control_methods_[j])
        {
          case POSITION:
            joint_control_methods_[j] = POSITION_PID;
            break;
          case VELOCITY:
            joint_control_methods_[j] = VELOCITY_PID;
            break;
        }
      }
      else
      {
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
#if GAZEBO_MAJOR_VERSION > 2
        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
#else
        joint->SetMaxForce(0, joint_effort_limits_[j]);
#endif
      }
    }
  }

  // Register interfaces
  super_hw_->registerInterface(&js_interface_);
  super_hw_->registerInterface(&ej_interface_);
  super_hw_->registerInterface(&pj_interface_);
  super_hw_->registerInterface(&vj_interface_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void DefaultRobotHwSim::readSim(ros::Time time, ros::Duration period)
{
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
    double position = sim_joints_[j]->Position(0);
#else
    double position = sim_joints_[j]->GetAngle(0).Radian();
#endif
    if (joint_types_[j] == urdf::Joint::PRISMATIC)
    {
      joint_position_[j] = position;
    }
    else
    {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                            position);
    }
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }
}

void DefaultRobotHwSim::writeSim(ros::Time time, ros::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  for(unsigned int j=0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
#if GAZEBO_MAJOR_VERSION >= 9
        sim_joints_[j]->SetPosition(0, joint_position_command_[j], true);
#else
        sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#endif
        break;

      case POSITION_PID:
        {
          double error;
          switch (joint_types_[j])
          {
            case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                            joint_position_command_[j],
                                                            joint_lower_limits_[j],
                                                            joint_upper_limits_[j],
                                                            error);
              break;
            case urdf::Joint::CONTINUOUS:
              error = angles::shortest_angular_distance(joint_position_[j],
                                                        joint_position_command_[j]);
              break;
            default:
              error = joint_position_command_[j] - joint_position_[j];
          }

          const double effort_limit = joint_effort_limits_[j];
          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case VELOCITY:
#if GAZEBO_MAJOR_VERSION > 2
        if (physics_type_.compare("dart") == 0)
        {
          sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
        }
        else 
        {
          sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
        }
#else
        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
#endif
        break;

      case VELOCITY_PID:
        double error;
        if (e_stop_active_)
          error = -joint_velocity_[j];
        else
          error = joint_velocity_command_[j] - joint_velocity_[j];
        const double effort_limit = joint_effort_limits_[j];
        const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                    -effort_limit, effort_limit);
        sim_joints_[j]->SetForce(0, effort);
        break;
    }
  }
}

void DefaultRobotHwSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void DefaultRobotHwSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}

////////////////////////////////////////
using namespace hector_quadrotor_interface;

QuadrotorHardwareSim::QuadrotorHardwareSim(HectorQuadrotorGimbalHwSim *super_hw)
{
  super_hw_ = super_hw;

  super_hw_->registerInterface(&interface_);
  interface_.registerAccel(&acceleration_);
  interface_.registerPose(&pose_);
  interface_.registerMotorStatus(&motor_status_);
  interface_.registerSensorImu(&imu_);
  interface_.registerTwist(&twist_);

  wrench_output_ = interface_.addInput<WrenchCommandHandle>("wrench");
  motor_output_ = interface_.addInput<MotorCommandHandle>("motor");
}

QuadrotorHardwareSim::~QuadrotorHardwareSim()
{

}

bool QuadrotorHardwareSim::initSim(
    const std::string &robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // store parent model pointer
  model_ = parent_model;
  link_ = model_->GetLink();
#if (GAZEBO_MAJOR_VERSION >= 8)
  physics_ = model_->GetWorld()->Physics();
#else
  physics_ = model_->GetWorld()->GetPhysicsEngine();
#endif

  model_nh.param<std::string>("world_frame", world_frame_, "world");
  model_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  // subscribe state
  std::string state_topic;
  model_nh.getParam("state_topic", state_topic);
  if (!state_topic.empty())
  {
    odom_sub_helper_ = boost::make_shared<OdomSubscriberHelper>(model_nh, state_topic, boost::ref(pose_),
                                                                boost::ref(twist_), boost::ref(acceleration_),
                                                                boost::ref(header_));
    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << state_topic << "' as state input for control" <<
    std::endl;
  }
  else
  {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as state input for control" <<
    std::endl;
  }

  // subscribe imu
  std::string imu_topic;
  model_nh.getParam("imu_topic", imu_topic);
  if (!imu_topic.empty())
  {
    imu_sub_helper_ = boost::make_shared<ImuSubscriberHelper>(model_nh, imu_topic, boost::ref(imu_));
    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << imu_topic << "' as imu input for control" <<
    std::endl;
  }
  else
  {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as imu input for control" <<
    std::endl;
  }

  motor_status_.on = true;
  motor_status_.header.frame_id = base_link_frame_;

  enable_motors_server_ = model_nh.advertiseService("enable_motors", &QuadrotorHardwareSim::enableMotorsCallback, this);

  wrench_limiter_.init(model_nh, "wrench_limits");

  wrench_command_publisher_ = model_nh.advertise<geometry_msgs::WrenchStamped>("command/wrench", 1);
  motor_command_publisher_ = model_nh.advertise<geometry_msgs::WrenchStamped>("command/motor", 1);

  return true;
}

void QuadrotorHardwareSim::readSim(ros::Time time, ros::Duration period)
{
  // read state from Gazebo
  const double acceleration_time_constant = 0.1;
#if (GAZEBO_MAJOR_VERSION >= 8)
  gz_acceleration_ = ((link_->WorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((link_->WorldLinearVel() - gz_angular_velocity_) + acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = link_->WorldPose();
  gz_velocity_ = link_->WorldLinearVel();
  gz_angular_velocity_ = link_->WorldAngularVel();
#else
  gz_acceleration_ = ((link_->GetWorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((link_->GetWorldLinearVel() - gz_angular_velocity_) + acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = link_->GetWorldPose();
  gz_velocity_ = link_->GetWorldLinearVel();
  gz_angular_velocity_ = link_->GetWorldAngularVel();
#endif

  // Use when Gazebo patches accel = 0 bug
//    gz_acceleration_ = link_->GetWorldLinearAccel();
//    gz_angular_acceleration_ = link_->GetWorldAngularAccel();

  if (!odom_sub_helper_)
  {
    header_.frame_id = world_frame_;
    header_.stamp = time;
#if (GAZEBO_MAJOR_VERSION >= 8)
    pose_.position.x = gz_pose_.Pos().X();
    pose_.position.y = gz_pose_.Pos().Y();
    pose_.position.z = gz_pose_.Pos().Z();
    pose_.orientation.w = gz_pose_.Rot().W();
    pose_.orientation.x = gz_pose_.Rot().X();
    pose_.orientation.y = gz_pose_.Rot().Y();
    pose_.orientation.z = gz_pose_.Rot().Z();
    twist_.linear.x = gz_velocity_.X();
    twist_.linear.y = gz_velocity_.Y();
    twist_.linear.z = gz_velocity_.Z();
    twist_.angular.x = gz_angular_velocity_.X();
    twist_.angular.y = gz_angular_velocity_.Y();
    twist_.angular.z = gz_angular_velocity_.Z();
    acceleration_.linear.x = gz_acceleration_.X();
    acceleration_.linear.y = gz_acceleration_.Y();
    acceleration_.linear.z = gz_acceleration_.Z();
    acceleration_.angular.x = gz_angular_acceleration_.X();
    acceleration_.angular.y = gz_angular_acceleration_.Y();
    acceleration_.angular.z = gz_angular_acceleration_.Z();
#else
    pose_.position.x = gz_pose_.pos.x;
    pose_.position.y = gz_pose_.pos.y;
    pose_.position.z = gz_pose_.pos.z;
    pose_.orientation.w = gz_pose_.rot.w;
    pose_.orientation.x = gz_pose_.rot.x;
    pose_.orientation.y = gz_pose_.rot.y;
    pose_.orientation.z = gz_pose_.rot.z;
    twist_.linear.x = gz_velocity_.x;
    twist_.linear.y = gz_velocity_.y;
    twist_.linear.z = gz_velocity_.z;
    twist_.angular.x = gz_angular_velocity_.x;
    twist_.angular.y = gz_angular_velocity_.y;
    twist_.angular.z = gz_angular_velocity_.z;
    acceleration_.linear.x = gz_acceleration_.x;
    acceleration_.linear.y = gz_acceleration_.y;
    acceleration_.linear.z = gz_acceleration_.z;
    acceleration_.angular.x = gz_angular_acceleration_.x;
    acceleration_.angular.y = gz_angular_acceleration_.y;
    acceleration_.angular.z = gz_angular_acceleration_.z;
#endif
  }

  if (!imu_sub_helper_)
  {
#if (GAZEBO_MAJOR_VERSION >= 8)
    imu_.orientation.w = gz_pose_.Rot().W();
    imu_.orientation.x = gz_pose_.Rot().X();
    imu_.orientation.y = gz_pose_.Rot().Y();
    imu_.orientation.z = gz_pose_.Rot().Z();

    ignition::math::Vector3d gz_angular_velocity_body = gz_pose_.Rot().RotateVectorReverse(gz_angular_velocity_);
    imu_.angular_velocity.x = gz_angular_velocity_body.X();
    imu_.angular_velocity.y = gz_angular_velocity_body.Y();
    imu_.angular_velocity.z = gz_angular_velocity_body.Z();

    ignition::math::Vector3d gz_linear_acceleration_body = gz_pose_.Rot().RotateVectorReverse(
        gz_acceleration_ - model_->GetWorld()->Gravity());
    imu_.linear_acceleration.x = gz_linear_acceleration_body.X();
    imu_.linear_acceleration.y = gz_linear_acceleration_body.Y();
    imu_.linear_acceleration.z = gz_linear_acceleration_body.Z();
#else
    imu_.orientation.w = gz_pose_.rot.w;
    imu_.orientation.x = gz_pose_.rot.x;
    imu_.orientation.y = gz_pose_.rot.y;
    imu_.orientation.z = gz_pose_.rot.z;

    gazebo::math::Vector3 gz_angular_velocity_body = gz_pose_.rot.RotateVectorReverse(gz_angular_velocity_);
    imu_.angular_velocity.x = gz_angular_velocity_body.x;
    imu_.angular_velocity.y = gz_angular_velocity_body.y;
    imu_.angular_velocity.z = gz_angular_velocity_body.z;

    gazebo::math::Vector3 gz_linear_acceleration_body = gz_pose_.rot.RotateVectorReverse(
        gz_acceleration_ - physics_->GetGravity());
    imu_.linear_acceleration.x = gz_linear_acceleration_body.x;
    imu_.linear_acceleration.y = gz_linear_acceleration_body.y;
    imu_.linear_acceleration.z = gz_linear_acceleration_body.z;
#endif
  }
}

void QuadrotorHardwareSim::writeSim(ros::Time time, ros::Duration period)
{
  bool result_written = false;

  if (motor_output_->connected() && motor_output_->enabled()) {
    motor_command_publisher_.publish(motor_output_->getCommand());
    result_written = true;
  }

  if (wrench_output_->connected() && wrench_output_->enabled()) {
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = time;
    wrench.header.frame_id = base_link_frame_;

    if (motor_status_.on && motor_status_.running) {
      wrench.wrench = wrench_limiter_(wrench_output_->getCommand());

      if (!result_written) {
#if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Vector3d force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
        ignition::math::Vector3d torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
#else
        gazebo::math::Vector3 force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
        gazebo::math::Vector3 torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
#endif
        link_->AddRelativeForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
        link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
#else
        link_->AddRelativeTorque(torque - link_->GetInertial()->GetCoG().Cross(force));
#endif
      }

    } else {
      wrench.wrench = geometry_msgs::Wrench();
    }

    wrench_command_publisher_.publish(wrench);
  }
}

bool QuadrotorHardwareSim::enableMotorsCallback(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res)
{
  res.success = enableMotors(req.enable);
  return true;
}

bool QuadrotorHardwareSim::enableMotors(bool enable)
{
  motor_status_.running = enable;
  return true;
}

} // ros_control_exercise

PLUGINLIB_EXPORT_CLASS(ros_control_exercise::HectorQuadrotorGimbalHwSim, gazebo_ros_control::RobotHWSim)

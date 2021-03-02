ros_control_exercise
====

This repository has samples to use ros_control.

# Real robot example
TBU

# Gazebo example
Example of hardware_inteface plugin for hector_quadrotor + camera-gimbal.

Plugin "libgazebo_ros_control" which is written into URDF is used to apply ros_control to gazebo simulation.
The plugin accepts subclass "gazebo_ros_control/RobotHWSim" and uses it to connect joints in gazebo to controllers of ros_control.
The subclass represents hardware-interface, and as default, `gazebo_ros_control/DefaultRobotSim` is used.
The default hw-interface can treat fundamental hw-interfaces such as `hardware_interface/EffortJointInterface`.
However, for example, hector_quadrotor_controller_gazebo package provides the original hw-interface `hector_quadrotor_controller_gazebo/QuadrotorHardwareSim`.
And this interfaces does not include the fundamental hw-interfaces.

In this example, new hw-interface subclass is provided, which combines `gazebo_ros_control/DefaultRobotSim` and `hector_quadrotor_controller_gazebo/QuadrotorHardwareSim`.

This part of this package is based on below packages.

* [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
* [tu-darmstadt-ros-pkg/hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor)

```
$ # install packages related with ros_control, gazebo_ros_controll
$ # install hector_quadrotor packages(For ROS-noetic, binary is not released, so please clone and build them)
$ roslaunch ros_control_exercise gimbal_gazebo_simulation.launch
$ # Test control of UAV and gimbal
$ rostopic pub /gimbal_sample/pitch_position_controller/command std_msgs/Float64 "data: 1.0"  # Gimbal pitch angle control
$ rostopic pub /action/pose/goal hector_uav_msgs/PoseActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'world'
      position:
        x: 0.0
        y: 0.0
        z: 5.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0"
```

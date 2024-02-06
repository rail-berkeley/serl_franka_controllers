/*
Refered to Source file:
  https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/joint_position_example_controller.cpp
*/

#include <serl_franka_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace serl_franka_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::vector<double> target_positions;
  if (!node_handle.getParam("/target_joint_positions", target_positions) || target_positions.size() != 7) {
    ROS_ERROR("JointPositionController: Could not read target joint positions from parameter server or incorrect size");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    reset_pose_[i] = target_positions[i];
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  for (size_t i = 0; i < 7; ++i) {
    if (elapsed_time_.toSec() > 10){
      position_joint_handles_[i].setCommand(reset_pose_[i]); // - delta_angle);
    }
    else {
      position_joint_handles_[i].setCommand( ((elapsed_time_.toSec()) / 10.0) * reset_pose_[i] + ((10 - elapsed_time_.toSec()) / 10.0) * initial_pose_[i]);
    }
  }
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerBase)
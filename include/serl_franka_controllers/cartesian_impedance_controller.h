// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <serl_franka_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <franka_msgs/ZeroJacobian.h>
#include <franka_msgs/FrankaDebug.h>

namespace serl_franka_controllers {
    // TODO:change to `serl_franka_controllers`

class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::array<double, 42> jacobian_array;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> Ki;
  Eigen::Matrix<double, 6, 6> Ki_target_;
  double translational_clip_neg_x;
  double translational_clip_neg_y;
  double translational_clip_neg_z;
  double translational_clip_x;
  double translational_clip_y;
  double translational_clip_z;
  double rotational_clip_x;
  double rotational_clip_y;
  double rotational_clip_z;
  double rotational_clip_neg_x;
  double rotational_clip_neg_y;
  double rotational_clip_neg_z;
  Eigen::Matrix<double, 3, 1> translational_clip_min;
  Eigen::Matrix<double, 3, 1> translational_clip_max;
  Eigen::Matrix<double, 3, 1> rotational_clip_min;
  Eigen::Matrix<double, 3, 1> rotational_clip_max;
  double rotational_clip;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> qe;
  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Vector3d position_d_;
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> error_i;
  Eigen::Vector3d position; 
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<serl_franka_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(serl_franka_controllers::compliance_paramConfig& config,
                               uint32_t level);
  void publishZeroJacobian(const ros::Time& time);
  realtime_tools::RealtimePublisher<franka_msgs::ZeroJacobian> publisher_franka_jacobian_;
  void publishDebug(const ros::Time& time);
  realtime_tools::RealtimePublisher<franka_msgs::FrankaDebug> publisher_franka_debug_;
  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace serl_franka_controllers

// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * ForwardTorqueController
 *
 * Subscribes to:  forward_torque_controller/commands  (std_msgs/Float64MultiArray)
 * Expects:        7 elements (torques for joints 1..7)
 * Outputs:        effort commands directly to the hardware interfaces
 */
class ForwardTorqueController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr int kNumJoints = 7;

  std::string arm_id_;
  std::string robot_description_;

  // Latest command (RT-safe)
  realtime_tools::RealtimeBuffer<Vector7d> torque_command_buffer_;
  Vector7d last_command_;
  bool have_command_{false};

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;

  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace franka_example_controllers
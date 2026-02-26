// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <franka_example_controllers/forward_torque_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <exception>
#include <string>
#include <vector>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
ForwardTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ForwardTorqueController::state_interface_configuration() const {
  // This controller does not require any state interfaces.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

CallbackReturn ForwardTorqueController::on_init() {
  try {
    // Keep consistent with the Franka examples: declare arm_id (even if we later overwrite it
    // from robot_description via robot_utils).
    auto_declare<std::string>("arm_id", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  last_command_.setZero();
  torque_command_buffer_.writeFromNonRT(last_command_);
  have_command_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForwardTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  // Fetch robot_description like the example controller, then derive the real arm_id_ from it.
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  // Overwrite arm_id_ from URDF (matches example behavior)
  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // Subscription: forward_torque_controller/commands
  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "forward_torque_controller/commands",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ForwardTorqueController::commandCallback, this, std::placeholders::_1));

  last_command_.setZero();
  torque_command_buffer_.writeFromNonRT(last_command_);
  have_command_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForwardTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // On activation, command zero torques until a message arrives.
  last_command_.setZero();
  torque_command_buffer_.writeFromNonRT(last_command_);
  have_command_ = false;

  // Also write zeros immediately once (nice to avoid stale values)
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForwardTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // On deactivation, send zero torques (best-effort).
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

void ForwardTorqueController::commandCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (!msg) {
    return;
  }
  if (msg->data.size() != static_cast<size_t>(kNumJoints)) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "forward_torque_controller: expected %d torques, got %zu. Ignoring.",
        kNumJoints, msg->data.size());
    return;
  }

  Vector7d cmd;
  for (int i = 0; i < kNumJoints; ++i) {
    cmd(i) = msg->data[i];
  }

  torque_command_buffer_.writeFromNonRT(cmd);
  have_command_ = true;
}

controller_interface::return_type ForwardTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // Read latest command (RT-safe). If no command received yet, this remains zeros.
  const Vector7d cmd = *(torque_command_buffer_.readFromRT());

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(cmd(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForwardTorqueController,
                       controller_interface::ControllerInterface)
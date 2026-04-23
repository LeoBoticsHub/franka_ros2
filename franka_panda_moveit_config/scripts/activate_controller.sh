#!/bin/bash
# Wait significantly longer for the hardware interface and motion generator to fully initialize
# This prevents acceleration discontinuity errors when the controller activates
echo "Waiting 12 seconds for hardware interface to stabilize..."
sleep 12

# Activate the arm controller
echo "Activating panda_arm_controller..."
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['panda_arm_controller']}" || echo "Failed to activate controller"


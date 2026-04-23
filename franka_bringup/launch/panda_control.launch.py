from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
  initial_joint_controller = LaunchConfiguration("initial_joint_controller")
  activate_joint_controller = LaunchConfiguration("activate_joint_controller")
  use_fake_hardware = LaunchConfiguration("use_fake_hardware")
  controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")

  robot_description_content = Command([
      PathJoinSubstitution([FindExecutable(name='xacro')]),
      " ", 
      PathJoinSubstitution([
          FindPackageShare("franka_description"), 
          "robots", "fer", "fer.urdf.xacro"
      ]),
      " ", 
      "use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
      " ",
      "robot_ip:=", LaunchConfiguration("robot_ip"),
        " ",
        "hand:=", "true",
        " ",
        "fake_sensor_commands:=", LaunchConfiguration("use_fake_hardware"),
        " ",
        "ros2_control:=", "true",

  ])
  robot_description = {
      'robot_description': ParameterValue(robot_description_content, value_type=str)
  }

  controllers_config = PathJoinSubstitution([FindPackageShare("franka_bringup"),
   "config","controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[controllers_config,robot_description],
    output="screen"
    )
  
  # Spawn controllers
  def controller_spawner(controllers, active=True):
    inactive_flags = ["--inactive"] if not active else []
    return Node(
          package="controller_manager",
          executable="spawner",
          arguments=[
              "--controller-manager",
              "/controller_manager",
              "--controller-manager-timeout",
              controller_spawner_timeout,
          ]
          + inactive_flags
          + controllers,
    )

  controllers_active = [
        "joint_state_broadcaster",
    ]
  controller_spawners = [
        controller_spawner(controllers_active),
        # controller_spawner(controllers_inactive, active=False),
    ]

  # joint_trajectory_controller = Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["joint_trajectory_controller", 
  #              "--controller-manager", "/controller_manager"],
  #   output='screen',
  # )

  # joint_state_broadcaster_spawner = Node(
  #   package="controller_manager",
  #   executable="spawner",
  #   arguments=["joint_state_broadcaster",
  #              "--controller-manager","/controller_manager"],
  #   output='screen',
  # )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description]
  )



  what_to_launch = [
    controller_manager_node,
    # joint_state_broadcaster_spawner,
    # joint_trajectory_controller,
    robot_state_publisher_node,
    ] + controller_spawners

  return what_to_launch

def generate_launch_description():
  launch_args = []
  launch_args.append(DeclareLaunchArgument(name="use_fake_hardware", default_value="true", description="use fake hardware"))
  launch_args.append(DeclareLaunchArgument(name="robot_ip", default_value="0.0.0.0", description="Robot ip"))
  launch_args.append(DeclareLaunchArgument(name="initial_joint_controller", default_value="joint_trajectory_controller", 
                                           description="Initial joint controller to be activated"))
  launch_args.append(DeclareLaunchArgument(name="activate_joint_controller", default_value="true", 
                                           description="Whether to activate the initial controller or not"))
  launch_args.append(DeclareLaunchArgument(name="controller_spawner_timeout", default_value="10", 
                                           description="Timeout for controller spawner"))

  ld = LaunchDescription(launch_args+[OpaqueFunction(function=launch_setup)])
    
  return ld

  
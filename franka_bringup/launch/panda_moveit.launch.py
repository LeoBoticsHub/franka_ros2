from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="use_fake_hardware", default_value="true", description="use use_fake_hardware hardware"),
    DeclareLaunchArgument(name="robot_ip", default_value="192.168.0.201", description="Robot ip"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  launch_moveit_path = PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka_moveit.launch.py'])
  launch_moveit_and_robot_description_launch = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments = [('use_fake_hardware', LaunchConfiguration("use_fake_hardware"))]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'panda_control.launch.py'])
  launch_controllers_launch  = IncludeLaunchDescription(
    launch_description_source = PythonLaunchDescriptionSource(launch_controllers_path),
    launch_arguments = [('use_fake_hardware', LaunchConfiguration("use_fake_hardware")),
                        ('robot_ip', LaunchConfiguration("robot_ip"))]
  )

  return [
    launch_moveit_and_robot_description_launch,
    launch_controllers_launch,
  ]
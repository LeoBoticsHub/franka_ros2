# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext
from launch_ros.actions import Node, PushRosNamespace


def namespace_controller_config(config_path, namespace):
    if not namespace:
        return config_path

    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)

    namespaced_config = {}
    namespace = namespace.strip('/')
    for node_name, node_config in config.items():
        namespaced_config[f'/{namespace}/{node_name}'] = node_config

    temp_file = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'franka_{namespace}_gazebo_controllers_',
        suffix='.yaml',
        delete=False)
    yaml.safe_dump(namespaced_config, temp_file)
    temp_file.close()
    return temp_file.name


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand, namespace):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    namespace_str = context.perform_substitution(namespace).strip('/')

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file, 
        mappings={
            'arm_id': arm_id_str, 
            'hand': load_gripper_str, 
            'ros2_control': 'true', 
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
        }
    )

    controllers_config = namespace_controller_config(
        os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'config',
            'franka_gazebo_controllers.yaml'),
        namespace_str)

    for plugin in robot_description_config.getElementsByTagName('plugin'):
        if plugin.getAttribute('name') == 'ign_ros2_control::IgnitionROS2ControlPlugin':
            for parameters in plugin.getElementsByTagName('parameters'):
                if parameters.firstChild is not None:
                    parameters.firstChild.data = controllers_config

            if namespace_str:
                ros = robot_description_config.createElement('ros')
                namespace_tag = robot_description_config.createElement('namespace')
                namespace_tag.appendChild(
                    robot_description_config.createTextNode(f'/{namespace_str}'))
                ros.appendChild(namespace_tag)
                plugin.appendChild(ros)

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    return [robot_state_publisher]


def launch_setup(context):
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)
    use_rviz = LaunchConfiguration('use_rviz')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    namespace_str = namespace.perform(context).strip('/')
    controller_manager_name = (
        f'/{namespace_str}/controller_manager' if namespace_str else '/controller_manager')
    robot_description_topic = (
        f'/{namespace_str}/robot_description' if namespace_str else '/robot_description')
    joint_states_topic = f'/{namespace_str}/joint_states' if namespace_str else 'joint_states'

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand, namespace])

    # Gazebo Sim
    # TODO: This launches gazebo in headless mode. To Fix
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r -s', 'ign_args': 'empty.sdf -r -s'}.items(),
    )


    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', robot_description_topic],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file, '-f', 'world'],
             condition=IfCondition(use_rviz),
             remappings=[
                 ('/tf', 'tf'),
                 ('/tf_static', 'tf_static'),
                 ('/robot_description', robot_description_topic),
             ],
    )
    
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager_name,
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
        output='screen'
    )
    
    forward_torque_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_torque_controller',
            '--controller-manager', controller_manager_name,
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
        output='screen'
    )

    namespaced_actions = [
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[forward_torque_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(use_joint_state_publisher),
            parameters=[
                {'source_list': [joint_states_topic],
                 'rate': 30}],
        ),
    ]

    if namespace_str:
        namespaced_actions = [
            GroupAction([
                PushRosNamespace(namespace_str),
                *namespaced_actions,
            ])
        ]

    return [gazebo_empty_world, *namespaced_actions]

def generate_launch_description():
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('franka_description')))

    launch_args = [
        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
            description='true/false for activating the gripper'),
        DeclareLaunchArgument(
            'franka_hand',
            default_value='franka_hand',
            description='Default value: franka_hand'),
        DeclareLaunchArgument(
            'arm_id',
            default_value='fer',
            description='Available values: fr3, fp3 and fer'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional robot namespace'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='true',
            description='Start joint_state_publisher'),
    ]

    return LaunchDescription([
        *launch_args,
        set_env_vars_resources,
        OpaqueFunction(function=launch_setup),
    ])

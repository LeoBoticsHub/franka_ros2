# ROS 2 Integration for Franka Robotics Research Robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

**Note:** franka_ros2 is not officially supported on Windows.



## Table of Contents
- [About](#about)
- [Caution](#caution)
- [Prerequisites](#prerequisites)
- [Optional .bashrc Settings](#optional-bashrc-settings)
- [Libfranka Installation](#libfranka-installation)
- [Setup](#setup)
  - [Install From Source](#install-from-source)
  - [Use VSCode DevContainer](#use-vscode-devcontainer)
  - [Test the Setup](#test-the-setup)
  - [Running on Real Hardware](#running-on-real-hardware)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
  - [Running Without Realtime Kernel](#running-without-realtime-kernel)
- [Services](#services)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

## Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankaemika/franka_ros2/issues).

## Prerequisites
Before installing **franka_ros2**, ensure you have the following prerequisites:
- **ROS 2 Humble Installation:** You can install [`ros-humble-desktop`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  or use VSCode IDE with DevContainer. 
- **PREEMPT_RT Kernel (optional but recommended):** A real-time kernel is necessary for the cartesian_pose, joint_position, and elbow_position command interfaces.
Informations on how to use a real-time kernel on a Linux machine can be found [at this link](https://ubuntu.com/blog/enable-real-time-ubuntu).
- **System-wide libfranka Installation:** 
    - If you plan to **install from source**, a libfranka installation is required. Please refer to the [libfranka repository](https://github.com/frankaemika/libfranka) for detailed build steps.
    This repository has been tested to work with the following versions of libfranka:
      - 0.9.2
      - 0.9.3
    **Attention:** Version 0.9.3 is not an official release of libfranka. Instructions on how to install this version are given in the [Libfranka Installation](#libfranka-installation) of this readme.
    - If you are **using the DevContainer**, you do not need to install libfranka system-wide, as it will be included in the container.

    Regardless of your setup, it is important to check the compatibility of your Robot OS version with libfranka to avoid potential errors. For detailed compatibility information, please consult the [libfranka compatibility table](https://frankarobotics.github.io/docs/libfranka/docs/compatibility_with_images.html).

## Optional .bashrc Settings
Enhance your development experience by adding the following line to your `.bashrc` file:

```bash
# Enable colorized warn and error messages
export RCUTILS_COLORIZED_OUTPUT=1
```

## Libfranka Installation
A system-wide libfranka installation is required. To communicate with the real robot, either version 0.9.2 or 0.9.3 of libfranka is
required as a first step.

### Installing ``libfranka 0.9.2``

1. Clone the ``0.9.2`` version of the libfranka repository on your PC:
```bash
git clone https://github.com/frankarobotics/libfranka.git
git checkout 0.9.2
git submodule init
git submodule update
```

2. Build Libfranka
```bash
cd libfranka
mkdir build && cd build
cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/opt/openrobots/lib -DCMAKE_PREFIX_PATH=/opt/openrobots/ ..
make franka -j$(nproc)
cpack -G DEB
sudo dpkg -i libfranka*.deb
sudo mae install
```

This should setup a system-wide installation of libfranka 0.9.2.

**Note:** Using libfranka 0.9.2 may lead to some missing dependencies issue. If this happens, refer to the [Troubleshooting](#troubleshooting) section of this readme.

### Installing ``libfranka 0.9.3``
As this is not an official release of libfranka, the user can install this version as described in the [Setup](#setup) section of this readme.

## Setup

### Install From Source

1. **Install Required Packages:**
   ```bash
   sudo apt install -y \
   ros-humble-ament-cmake \
   ros-humble-ament-cmake-clang-format \
   ros-humble-angles \
   ros-humble-ros2-controllers \
   ros-humble-ros2-control \
   ros-humble-ros2-control-test-assets \
   ros-humble-controller-manager \
   ros-humble-control-msgs \
   ros-humble-control-toolbox \
   ros-humble-generate-parameter-library \
   ros-humble-joint-state-publisher \
   ros-humble-joint-state-publisher-gui \
   ros-humble-moveit \
   ros-humble-pinocchio \
   ros-humble-realtime-tools \
   ros-humble-xacro \
   ros-humble-hardware-interface \
    ros-humble-ros-gz \
   python3-colcon-common-extensions
   ```


2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   ```
3. **Clone the Repositories:**
   ```bash
    source /opt/ros/humble/setup.bash
    cd ~/franka_ros2_ws 
    git clone https://github.com/LeoBoticsHub/franka_ros2.git -b devel_alessandro src/franka_ros2
    ```

4. **Install ``libfranka 0.9.3``:**
    Skip this step if you already installed ``libfranka 0.9.2``.

    ```bash
    cd ~/franka_ros2_ws/src/franka_ros2/libfranka-active-control
    mkdir build && cd build
    cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/opt/openrobots/lib -DCMAKE_PREFIX_PATH=/opt/openrobots/ ..
    make franka -j$(nproc)
    cpack -G DEB
    sudo dpkg -i libfranka*.deb
    sudo make install
    ```
5. **Build and Source the Workspace**
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    source install/setup.bash
    ```
### Use VSCode DevContainer
The `franka_ros2` package includes a `.devcontainer` folder, which allows you to use Franka ROS 2 packages without manually installing ROS 2 or `libfranka`. For detailed instructions, follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Create a ROS 2 Workspace:**
   ```bash
   mkdir franka_ros2_ws
   cd franka_ros2_ws
   ```

2. **Clone the Repositories:**
    ```bash
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 
    git clone https://github.com/frankaemika/franka_description.git src/franka_description
    ```

3. **Copy the .devcontainer Folder:**
    ```bash
    cp -r src/franka_ros2/.devcontainer .
    ```
    This step ensures that both the `franka_ros2` and `franka_description` folders are accessible within the DevContainer environment.

4. **Open VSCode:**
    ```bash
    code . 
    ```
5. **Open the Current Folder in DevContainer:**
    
    Press Ctrl + Shift + P and type: `Dev Containers: Rebuild and Reopen in Container`.


6. **Open the Terminal in VSCode:**
    
    Press Ctrl + (backtick). 

7. **Source the Environment:**
    ```bash
    source /opt/ros/humble/setup.sh  
    ```
8. **Install the Franka ROS 2 Packages:**
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
    source install/setup.sh 
    ```


### Test the Setup

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```

## Running on Real Hardware
To interface the ros2_control with the Franka Emika Panda Hardware one can refer to this [guide](https://github.com/LeoBoticsHub/franka_ros2/blob/devel_alessandro/Docs/Instructions/interface_with_hardware.md).



## Troubleshooting
### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankarobotics.github.io/docs/libfranka/docs/real_time_kernel.html).

### `Running Without Realtime Kernel`
If your PC performance are good enough there is a way of commanding the the robot without requiring a real-time kernel. To enable operation without real-time kernel you need to modify two scripts contained in `libfranka-active-control/src`:
- `control_loop.cpp`
- `robot_impl.h`

By commenting out certain lines of code and uncommenting others it is possible to bypass the realtime kernel need.
The two scripts already contain information about which lines need to be commented/uncommented so you can use an IDE to make the changes.
Once you do so, remember to re-build your workspace and source it or your changes won't take effect.

```bash
cd franka_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Disclaimer**: If your application is heavy or your PC is not performing, doing this may hinder communication leading to a robot safety stop.

## Services
Here are some useful ros2 services which one may need when runnning the robot.

### Set Full Collision Behavior

Particularly useful when the robot has to interact with the environment (e.g. Bilateral Teleoperation, continuous contact tasks, etc.).

Allows to change thresholds after which the robot goes into emergency stop:

```bash
# Example command to change the collision behavior
ros2 service call /service_server/set_full_collision_behavior franka_msgs/srv/SetFullCollisionBehavior "{
  lower_torque_thresholds_acceleration: [100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0],
  upper_torque_thresholds_acceleration: [100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0],
  lower_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0],
  upper_torque_thresholds_nominal: [100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 20.0],
  lower_force_thresholds_acceleration: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0],
  upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 10.0, 10.0, 10.0],
  lower_force_thresholds_nominal: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0],
  upper_force_thresholds_nominal: [30.0, 30.0, 30.0, 20.0, 20.0, 20.0]
}"
```


## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankaemika/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project. 


## License 

All packages of franka_ros2 are licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html). 
 

## Contact 

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankaemika/franka_ros2/issues) page. 

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more information.

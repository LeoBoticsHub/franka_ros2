# Interfacing ROS2 with the Franka Emika Panda via Franka Control Interface (FCI)
This guide helps you setup a connection and command Franka Emika Panda robots of Leonardo Innovation Hub using ROS2 and the Franka Control Interface (FCI).

## Contents
- [Powering On the Robot](#powering-on-the-robot)
- [Connection Setup](#connection-setup)
- [Franka Desk](#franka-desk)
- [External Buttons](#external-buttons)
- [Activating FCI](#activating-fci)
- [Run an Example Controller](#run-an-example-controller)

## Powering On the Robot
Two identical Franka Emika Panda robots are available for usage. Their respective control boxes are place underneath the table where they are mounted.
To power on one of the robots, simply press the start button placed at the back of the corresponding control box.

Once you do so, the two led strips at the base of the robot will start to blink yellow.
Once the leds stop blinking, the robot is ready for usage but its motors still need to be powered on and the brakes released.

This can be easily done through ``Franka Desk`` once we establish a connection between the robot and our pc.

## Connection Setup
The robots and your pc can be connected via ethernet as follows:
1. Connect the robot you want to control to your PC via the dedicated ethernet cable.
2. Set your PC's static IP to ``192.168.0.200``
3. Ping the robot you want to control to test the connection (IP addresses for the robots are found on the back of the base joint):
```bash
# Example for robot with IP 192.168.0.201
ping 192.168.0.201
```

Once this step is completed, you can proceed to releasing the brakes of the robot and activating the motors via Franka Desk.

## Franka Desk
Franka Desk is the application used to program and monitor the state of Franka robots.
For a complete guide on the usage of the desk and how it can be used to program Franka robots please refer to [Franka's Official Video Tutorials](https://www.youtube.com/playlist?list=PL9FoYHNFGS3TyHsLcL0-qNIi-e14Xw7np).

1. **Open Desk**:
Open a web browser (Desk is supported on Safari, Chrome and Firefox) and type in the search bar the IP of the robot to which you're connected.
This will open up the desk app on your pc.

2. **Release the Brakes**
Once you're logged in, you can release the brakes by clicking on the ``open lock`` icon found on the right under the joints tab.

You will hear a clicking sound which are the barkes being released. The robot leds will then turn either blue or white depending on the state of the external buttons as explained in the following section.

**Notes**
- If it's the first time you access the desk app, the system may require username and password. These can be found in the Leonardo Innovation Hub Github repository.
  If you don't have access to that part of the repository please ask your supervisor.
- While navigating to Desk, it may be necessary to bypass a security message telling you that the connection you want to establish is not secure.

## External Buttons
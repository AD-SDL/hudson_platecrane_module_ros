# platecrane_ros_packages

## Installation

```
mkdir ~/sciclops_ws
cd ~/sciclops_ws

mkdir src
cd src
git clone https://github.com/AD-SDL/platecrane_ros_packages
git clone https://github.com/AD-SDL/wei_ros
cd ..
colcon build
```

## Description

A repository for the Hudson Plate Stacker (Sciclops).

This package guides a user to remotely control the sciclops.

Sciclops is the main object for removing microplates from the stackers and placing them on the exchange platform.

Sciclops has 4 controllable axes and a gripper.
<p>&nbsp;</p>
        Z: Vertical axis <br>
        R: Base turning axis <br>
        Y: Extension axis <br>
        P: Gripper turning axis

<p>&nbsp;</p>

## Current Features
* Sciclops initialization
* Movements (move to preset points. ex: Neutral, Stack 1, etc.)
* Precise movements (move to certain point or certain distance)

## User Guide
1. ### Find and setup port
    * Connect Sciclops to device with a serial to usb cable
    * Run following code in python script

          file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/SCICLOPS_logs/robot_client_logs.log')

2. ### Create ROS2 infrastructure
     * Install ROS2 and create ROS2 workspace
        https://docs.ros.org/en/humble/Installation.html

3. ### Git clone hudson_driver repository
     * cd into src folder
In terminal:

           git clone https://github.com/AD-SDL/hudson_driver.git

4. ### Run Commands
     * In terminal

           colcon build

           source install/setup.bash

           ros2 run sciclops_ros_client sciclopsNode

     * Open new terminal

            ros2 service call /sciclops_actions sciclops_module_services/srv/SciclopsActions "{action_request: <command>}"

        #### Possible Commands
        * Get Plate 1
        * Get Plate 2
        * Get Plate 3
        * Get Plate 4
        * Get Plate 5
        * Home
        * Status

## Updating the code
   * cd into src folder
   * git pull
   * colcon build
   * source install/setup.bash

## Authors and Maintainers
* Sanjiv Parthasarathy
* Rafael Vescovi
* Eric Codrea
* Abe Stroka

# ROS2 CANopen

## Eirabot

### Notes: 
This project is based on the humble branch of the [ROS2 CANOpen Stack](https://github.com/ros-industrial/ros2_canopen/tree/humble) which is currently under development. As such, a number of workarounds have been implemented to get this demo up and running.

Problems with ```cogen``` (Custom cmake macro used to streamline bus.yml files) not working correctly.  ```generate_dcf``` is used instead. However, for this reason, two almost identical versions of the slave eds file must be used to account for different node ids. As well as this, hardcoded file locations are used for the time being, but could be done away with once ```cogen``` is working.

The project has been confirmed to run on a Kontron PC running a minimal installation of Ubuntu Server 22.04, however this leaves minimal hard disk space (approx. 100 Mb) after all dependencies are installed.

### File structure
Due to hardcoded file names, the project must be built in a directory called ```ros2_cows``` in the Kontron PC's home directory.

```
mkdir ros2_cows
cd ros2_cows
mkdir src
```

### Installing
[ROS2 Humble installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
Note: the ```ros-humble-desktop``` package is not necessary to run the demo 

Necessary modules:

```
sudo apt install net-tools vim can-utils ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-teleop-twist-keyboard
```


Clone ros2_canopen into your ```ros2_cows``` source folder and install dependencies

```
git clone -b eirabot git@github.com:eiratechrobotics/Eirabot-Ros.git
cd ..
rosdep install --from-paths src/ros2_canopen --ignore-src -r -y
```

### Building

The project can take a long time to build for the first time, especially on the Kontron PC (up to 20 mins).

In the ```ros2_cows``` directory run the following command:

```
colcon build
```

When working with the ```canopen_tests``` package in particular, the following command allows for quicker build times:

```
colcon build --packages-select canopen_tests
```

### Bring up

Start the CAN bus.
```
sudo ip link set can0 up type can bitrate 800000
```

Workaround: Project must first be run as root user to allow CAN permissions. Run as regular user after this.
```
sudo -i

cd /home/user/ros2_cows
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch canopen_tests eirabot.launch.py

exit
```

The same commands can be run again in order to launch the project as a regular user.

In another window run the following commands to put the robot in velocity mode and set acceleration and max velocity values:

```
ros2 service call /left_wheel_joint/velocity_mode std_srvs/srv/Trigger
ros2 service call /right_wheel_joint/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3380, subindex: 0, data: 18000}"
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3380, subindex: 0, data: 18000}"
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3321, subindex: 0, data: 800}"
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3323, subindex: 0, data: 800}"
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3321, subindex: 0, data: 800}"
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3323, subindex: 0, data: 800}"
```

The reason that these SDO writes are done manually is that they appear to be overwritten when setting the robot to velocity mode.

The following ```teleop_twist_keyboard``` command can then be used to drive the robot:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

### Todo
- Find reason for running as root user at first
- Improve responsiveness of motors
- Set motors to velocity mode by default and remove need for manual SDO writes
- Update ```bus.yml``` to use ```cogen``` when possible
- Update urdf to better approximate robot for simulation

## Status

| Build Process | Status |
|---------------|--------|
| Industrial CI Build | [![CI Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions) |
| Documentation Build | [![CI Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions) |
| Buildfarm Build | [![Buildfarm Status](https://build.ros2.org/job/Rdev__ros2_canopen__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__ros2_canopen__ubuntu_jammy_amd64/) |
[![humble](https://github.com/ros-industrial/ros2_canopen/actions/workflows/humble.yml/badge.svg)](https://github.com/ros-industrial/ros2_canopen/actions/workflows/humble.yml)
[![HUMBLE Documentation](https://github.com/ros-industrial/ros2_canopen/actions/workflows/humble_documentation.yml/badge.svg)](https://github.com/ros-industrial/ros2_canopen/actions/workflows/humble_documentation.yml)

The stack is currently under development and not yet ready for production use.

| Binary Package (rolling) | Jammy | Rhel9 |
|---------|-------|-------|
| canopen_interfaces | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_interfaces__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_interfaces__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_interfaces__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_interfaces__rhel_9_x86_64__binary/) |
| lely_core_libraries | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__lely_core_libraries__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__lely_core_libraries__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__lely_core_libraries__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__lely_core_libraries__rhel_9_x86_64__binary/) |
| canopen_core | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_core__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_core__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_core__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_core__rhel_9_x86_64__binary/) |
| canopen_master_driver | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_master_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_master_driver__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_master_driver__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_master_driver__rhel_9_x86_64__binary/) |
| canopen_base_driver | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_base_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_base_driver__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_base_driver__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_base_driver__rhel_9_x86_64__binary/) |
| canopen_proxy_driver | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_proxy_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_proxy_driver__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_proxy_driver__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_proxy_driver__rhel_9_x86_64__binary/) |
| canopen_402_driver | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_402_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_402_driver__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_402_driver__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_402_driver__rhel_9_x86_64__binary/) |
| canopen_ros2_control | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_ros2_control__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_ros2_control__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_ros2_control__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_ros2_control__rhel_9_x86_64__binary/) |
| canopen_ros2_control | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_ros2_controllers__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_ros2_controllers__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_ros2_controllers__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_ros2_controllers__rhel_9_x86_64__binary/) |
| canopen_tests | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_tests__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_tests__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_tests__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_tests__rhel_9_x86_64__binary/) |
| canopen_utils | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__canopen_utils__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__canopen_utils__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_rhel_el964__canopen_utils__rhel_9_x86_64__binary/badge/icon)](https://build.ros2.org/job/Rbin_rhel_el964__canopen_utils__rhel_9_x86_64__binary/) |

## Documentation
The documentation consists of two parts: a manual and an api reference.
The documentation is built for rolling (master), iron and humble and hosted on github pages.
Older ROS 2 releases are EOL and are not supported anymore.

### Rolling
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/rolling/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/rolling/

### Iron
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/iron/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/iron/

### Humble
* Manual: https://ros-industrial.github.io/ros2_canopen/manual/humble/
* API reference: https://ros-industrial.github.io/ros2_canopen/api/humble/

## Features
These are some of the features this stack implements. For further information please refer to the documentation.

* **YAML-Bus configuration**
  This canopen stack enables you to configure the bus using a YAML file. In this file you define the nodes that are connected to the bus by specifying their node id, the corresponding EDS file and the driver to run for the node. You can also specify further parameters that overwrite EDS parameters or are inputs to the driver.
* **Service based operation**
  The stack can be operated using standard ROS2 nodes. In this case the device container will load the drivers for master and slave nodes. Each driver will be visible as a
  node and expose a ROS 2 interface. All drivers are brought up when the device manager is launched.
* **Managed service based operation**
  The stack can be opeprated using managed ROS2 nodes. In
  this case the device container will load the drivers for master and slave nodes based on the bus configuration. Each driver will be a lifecycle node and expose a ROS 2 interface. The lifecycle manager can be used to bring all
  device up and down in the correct sequence.
* **ROS2 control based operation**
  Currently, multiple ros2_control interfaces are available. These can be used for controlling CANopen devices. The interfaces are:
  * canopen_ros2_control/CANopenSystem
  * canopen_ros2_control/CIA402System
  * canopen_ros2_control/RobotSystem
* **CANopen drivers**
  Currently, the following drivers are available:
    * ProxyDriver
    * Cia402Driver


## Post testing
To test stack after it was built from source you should first setup a virtual can network.
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
```
Then you can launch a managed example
```bash
ros2 launch canopen_tests cia402_lifecycle_setup.launch.py
ros2 lifecycle set /lifecycle_manager configure
ros2 lifecycle set /lifecycle_manager activate
```

Or you can launch a standard example
```bash
ros2 launch canopen_tests cia402_setup.launch.py
```

Or you can launch a ros2_control example
```bash
ros2 launch canopen_tests robot_control_setup.launch.py
```

## Contributing
This repository uses `pre-commit` for code formatting.
This program has to be setup locally and installed inside the repository.
For this execute in the repository folder following commands:
```
sudo apt install -y pre-commit
pre-commit install
```
The checks are automatically executed before each commit.
This helps you to always commit well formatted code.
To run all the checks manually use `pre-commit run -a` command.
For the other options check `pre-commit --help`.

In a case of an "emergency" you can avoid execution of pre-commit hooks by adding `-n` flag to `git commit` command - this is NOT recommended to do if you don't know what are you doing!

# Installation

## Windows
Running this package on windows requires the use of WSL (Ubuntu22.04). Visit [this page](https://learn.microsoft.com/en-us/windows/wsl/install) for more information on the installation process.

After installing ROS2 humble on WSL (see link below), clone this repository in WSL and navigate to `247Swarm/ros2_ws/` . Build the workspace by running:
```bash
colcon build --symlink-install
source ./install/setup.bash
```
Any time you edit launch or setup files, you are required to run these commands again.

If you are planning on using our collision avoidance, proceed with the following steps.

Move the `WindowsBuild` directory from `collision_avoidance` to somewhere outside of WSL and copy the path to the `WindowsCA_Data` directory. In [config.py](ros2_ws/src/swarm_operation/swarm_operation/config.py) scroll down to the end and change CA_CONFIG_DIR to `/mnt/YOUR_PATH`. For example, if you store the `WindowsBuild` folder in `c/Users/name/documents/` then CA_CONFIG_DIR has to be `/mnt/c/Users/name/documents/WindowsBuild/WindowsCA_Data`

You can remove the `LinuxBuild` directory.

&nbsp;

## Linux (Ubuntu22.04)
Clone this repository in WSL and navigate to `247Swarm/ros2_ws/` . Build the workspace (after installing ROS2, see link below) by running:
```bash
colcon build --symlink-install
source ./install/setup.bash
```
Any time you edit launch or setup files, you are required to run these commands again.

You can remove the `WindowsBuild` directory in `collision_avoidance`.

&nbsp;

## Dependencies

This package is built on ROS2 humble. Visit their [installation page](https://docs.ros.org/en/humble/) for more information on the installation process.

To control the Crazyflies through pyhton, cflib is used. Follow the [installation instructions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/install/) and set the [usb permissions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/). To have the usb permissions setup every time, add the following lines to your `.bashrc`:
```bash
sudo /etc/init.d/udev restart
sudo udevadm trigger
```
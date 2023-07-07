# 247Swarm

This is a ROS2 package to control a swarm of [Crazyflies](https://www.bitcraze.io/) for research or outreach purposes. Developed by [Emergent Swarms](https://www.linkedin.com/company/emergentswarms) for Delft University of Technology.

Using the [wireless charging](https://www.bitcraze.io/products/qi-1_2-charger-deck/) deck on the crazyflies, this package takes care of automatically returning the drones to a landing pad when their battery is empy or when instructed to do so. Using a position or velocity commander, you can implement swarming algorithms without having to think about the battery live of the drones. Our collision avoidance package is provided which allows for carefree implementations of swarming algorithms. The collision avoidance can be disabled in case you want to design your own.

## Table of contents

### [Installation](INSTALLATION.md)

### [Crazyflie setup](CRAZYFLIE_SETUP.md)

### [Lighthouse base station setup](LIGHTHOUSE_SETUP.md)

### [Landing pad setup](LANDING_PAD_SETUP.md)

### [Usage](USAGE.md)

### [Parts list](PARTS.md)
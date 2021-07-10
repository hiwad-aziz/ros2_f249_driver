
# F249 Wheel Encoder Driver for ROS2 and Raspberry Pi
This repository contains a ROS2 package that interfaces with the Raspberry Pi GPIOs to count the ticks sent out by an F249 sensor and calculate the RPM. The sensor alone is not capable of identifying the direction of rotation. In any case, it will accumulate the number of ticks for both rotation directions and calculate the RPM based on that. 

## Dependencies
-  [WiringPi](https://github.com/WiringPi/WiringPi/)

## Setup
Make sure the configuration parameters in f249-driver.h are set according to your setup.
Configurable parameters are:
- GPIO Pin connected to sensor
- Minimum time between two published messages
- Number of slots of encoder wheel

Build the package in your workspace:

    colcon build --packages-select f249-driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 run f249-driver f249-driver

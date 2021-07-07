
# F249 Wheel Encoder Driver for ROS2 and Raspberry Pi
This repository contains a ROS2 package that interfaces with the Raspberry Pi GPIOs to count the ticks sent out by an F249 sensor. The sensor alone is not capable of identifying the direction of rotation. In any case, it will accumulate the number of ticks for both rotation directions. It does not directly calculate rpm or velocity.

## Dependencies
-  [WiringPi](https://github.com/WiringPi/WiringPi/)

## Setup
Make sure the PIN constant in f249-driver.cpp is set to the PIN on your RPi that the sensor is connected to.

Build the package in your workspace:

    colcon build --packages-select f249-driver
    
Launch it:

    ros2 run f249-driver f249driver

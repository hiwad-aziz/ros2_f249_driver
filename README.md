# F249 Wheel Encoder Driver for ROS2 and Raspberry Pi
This repository contains a ROS2 package that interfaces with the Raspberry Pi GPIOs to count the ticks sent out by an F249 sensor. The sensor alone is not capable of identifying the direction of rotation. In any case, it will accumulate the number of ticks for both rotation directions.

## Dependencies
- [WiringPi](https://github.com/WiringPi/WiringPi/)
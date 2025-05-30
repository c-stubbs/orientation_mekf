# ROS2 MEKF

Simple MEKF orientation estimation in ROS2.

## Description

A simple Multiplicative Extended Kalman Filter (MEKF) class to estimate orientation using an inertial measurement unit (IMU). Integrated with ROS2, this class can subscribe to IMU information and generate a ROS2 transform message.

## Getting Started

### Dependencies

* ROS2

### Installing

* Clone this repository into the src directory of your ROS2 workspace
* Source ROS2 in your environment
```
source /opt/ros/jazzy/setup,bash
```
* Build the workspace
```
colcon build
```
* Source your workspace
```
source install/setup.bash
```

### Executing program

```
ros2 run mekf orientation_node
```

<!-- ## Help

Any advise for common problems or issues.
```
command to run if program contains helper info
``` -->

## Authors

[Chandler Stubbs](https://github.com/c-stubbs)

<!-- ## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release -->

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Thanks to Dr. Howard Chen and his dissertation for a thorough explanation of the real-world implementation of a MEKF for orientation estimation!
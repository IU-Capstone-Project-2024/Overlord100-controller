# Overlord100-controller
Controller submodule

`DiffDriveController` is a ROS2 node designed to control a differential drive robot. 

## Module structure 

`cmd_vel_publisher.cpp` and `controller_subscriber.cpp` are temporary files to test the controller (`controller.cpp`).

*cmd_vel_publisher (`cmd_vel_publisher.cpp`) sends `Twist` over the topic `cmd_vel` to the controller (`controller.cpp`). And the controller sends `WheelData` to the controller_subscriber (`controller_subscriber.cpp`) over the topic `wheels_control`*


## Features

- Subscribes to the `cmd_vel` topic (with the assumption that `cmd_vel` measurements unit is m/s).
- Publishes (angular) wheel velocities (in rpm) to the `wheels_control` topic.
- Uses parameters for platform base wheel radius (the robot's geometry).

### Topics 

**Subscribed**

- `/cmd_vel` (`geometry_msgs/msg/Twist`): Subscribes to the velocity command messages, which contain linear and angular velocities. 

- `/wheels_control` (`overlord100_msgs/msg/WheelsData`): Publishes the calculated wheel velocities to control the robot. 

**Published** 

### How to run the node:

```ros2 run controller diff_drive_controller```
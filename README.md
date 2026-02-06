# Swerve Robot

A simple swerve robot in Gazebo

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
device on which you want to run this code.

1. [ROS Jazzy](https://docs.ros.org/en/jazzy/Installation.html) with the following packages: robot state publisher, joint state broadcaster, robot localization, Nav2, Rviz2 and (GazeboSim Harmonic)
   `robot_state_publisher`, the `joint_state_broadcaster` and the
   [ros_control](https://control.ros.org/master/index.html) packages.
2. A working [ROS workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

## Usage

This package can be launched by using the command line:

    ros2 launch single_swerve_robot robot.launch.py

You can start with goal planning if the Nav2 plugin in Rviz2 shows Navigation and Localization as active

## Acknowledgement

This project incorporates code from https://github.com/ROBOTIS-GIT/ai_worker/tree/main

Only the folder ffw_swerve_drive_controller (https://github.com/ROBOTIS-GIT/ai_worker/tree/main/ffw_swerve_drive_controller) is used.



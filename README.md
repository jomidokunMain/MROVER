# Mrover

## History

- 12/30/2022: Version 1.0 released rover branch devel_mrover from [Oscar](https://github.com/jrkwon/oscar/tree/devel_mrover).
- 04/15/204: Version 2.0 released noetic
- 04/15/204: Version 2.0 released foxy in branch ros2_foxy

## Introduction

Mrover is upgraded version of the OSCAR rover providing a the Mesoscale Open-Source robotic Car Architecture for Research and education in a autonomous vehicle setting.

The OSCAR platform was designed in the Bio-Inspired Machine Intelligence Lab at the University of Michigan-Dearborn.

The OSCAR supports two vehicles: `fusion` and `Mrover`.

The backend system of `Mrover` is the PX4 Autopilot with Robotic Operating System (ROS) communicating with PX4 running on hardware or on the Gazebo simulator.

## Who is MROVER for?

The MROVER platform can be used by researchers who want to have a full-stack system for a robotic car that can be used in autonomous vehicles and mobile robotics research.
MOVER helps educators who want to teach mobile robotics and/or autonomous vehicles in the classroom.
Students also can use the OSCAR platform to learn the principles of robotics programming.

## Download the MROVER Source Code

```
$ git clone https://github.com/jomidokunMain/mrover.git --recursive
```

## Vehicle Control System

The are two different types of vehicle control system

- Four wheel differential
- Ackermann Steering Geometric

## MROVER Project Repository

Welcome to the MROVER project repository! Here you will find various projects related to the MROVER vehicle, ranging from basic setups to advanced applications. Whether you're interested in manual control, neural network integration, or advanced mapping techniques, this repository has something for you.

### Projects Overview

1. [**Vehicle Setup**](projects/vehicle_setup.md)

   - Set up the MROVER vehicle hardware and software environment.

2. [**Run Vehicle with Manual Control**](projects/manual_control.md)

   - Control the MROVER vehicle manually using a joystick or keyboard input.

3. [**Run Vehicle with OSCAR Neural Network**](projects/oscar_neural_network.md)

   - Implement the OSCAR neural network for autonomous navigation of the MROVER vehicle.

4. [**Run Vehicle with Camera-Based SLAM**](projects/camera_based_slam.md)

   - Utilize camera sensors for Simultaneous Localization and Mapping (SLAM) to navigate the MROVER vehicle.

5. [**Run Vehicle with LIDAR-Based SLAM**](projects/lidar_based_slam.md)

   - Implement LIDAR sensors for SLAM to enable precise mapping and navigation for the MROVER vehicle.

### Additional Projects

6. [**Obstacle Avoidance**](projects/obstacle_avoidance.md)

   - Develop algorithms to enable the MROVER vehicle to autonomously avoid obstacles in its path.

7. [**GPS Navigation Integration**](projects/gps_navigation_integration.md)

   - Integrate GPS navigation capabilities into the MROVER vehicle for outdoor navigation.

8. [**Machine Learning for Path Planning**](projects/machine_learning_path_planning.md)

   - Utilize machine learning algorithms to optimize path planning for the MROVER vehicle.

9. [**Multi-Vehicle Coordination**](projects/multi_vehicle_coordination.md)

   - Develop protocols for coordinating multiple MROVER vehicles to work together on a task.

### Getting Started

To get started with any of the projects, simply click on the project name to navigate to its respective README file. Each project directory contains detailed instructions on setting up and running the project.

### Contribution Guidelines

Contributions to this repository are welcome! If you have any improvements or new project ideas, feel free to submit a pull request following the contribution guidelines outlined in the `CONTRIBUTING.md` file.

### License

This project is licensed under the [MIT License](LICENSE).

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

The are two different type of vehicle control system

- Four wheel differential
- Ackermann Steering Geometric

# ROS2 Mobile Robot

## Overview

A ROS2-based mobile robot project developed as part of my Digital
Mechatronic Engineering coursework.

The project uses ROS2 nodes to process sensor data, control robot
motion and generate odometry.

## Technologies

- ROS2 Jazzy
- Python
- Linux / WSL
- RViz
- OpenCV

## System Architecture

The system consists of:

- Wheel encoder publisher
- Wheel tick to odometry node
- Colour detection node
- Robot velocity controller

## Features

- Publishes wheel encoder data
- Calculates robot odometry
- Processes camera images
- Detects colours
- Controls robot velocity using `/cmd_vel`

## What I Learned

- ROS2 publishers and subscribers
- ROS2 package structure
- Node communication
- Sensor data processing
- Robot odometry
- Debugging ROS2 systems
- 

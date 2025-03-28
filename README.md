# ROS 2 Bug 0 Algorithm Simulation for e-puck in Webots

This repository contains a ROS 2 package for simulating the Bug 0 algorithm using the e-puck robot within the Webots simulator. The package includes a custom world file with multiple obstacles, designed to test and demonstrate the Bug 0 navigation strategy.

## Table of Contents

- [Description](#description)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Custom World File](#custom-world-file)
- [Bug 0 Algorithm Implementation](#bug-0-algorithm-implementation)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Description

This ROS 2 package implements the Bug 0 algorithm for the e-puck robot in the Webots simulator. The Bug 0 algorithm is a simple navigation method that allows a robot to reach a goal while avoiding obstacles. The simulation environment includes a custom world file with various obstacles, providing a realistic scenario for testing the algorithm's performance.

## Prerequisites

Before running this package, ensure you have the following software installed:

- **ROS 2 (Galactic/Humble or later):** Follow the official ROS 2 installation instructions for your operating system.
- **Webots Simulator:** Install Webots from the Cyberbotics website. It is compatible with ROS 2.
- **e-puck Webots ROS 2 package:** This package provides the necessary ROS 2 interface for the e-puck robot in Webots. You can install it using:

   ```bash
   sudo apt install ros-<ros2-distro>-webots-ros2

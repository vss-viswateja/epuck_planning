# ROS 2 Bug 0 Algorithm Simulation for e-puck in Webots

This repository contains a ROS 2 package for simulating the Bug 0 algorithm using the e-puck robot within the Webots simulator. The package includes a custom world file with multiple obstacles, designed to test and demonstrate the Bug 0 navigation strategy.

## Table of Contents

- [Description](#description)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Description

This ROS 2 package implements the Bug 0 algorithm for the e-puck robot in the Webots simulator. The Bug 0 algorithm is a simple navigation method that allows a robot to reach a goal while avoiding obstacles. The simulation environment includes a custom world file with various obstacles, providing a realistic scenario for testing the algorithm's performance.

## Prerequisites

Before running this package, ensure you have the following software installed:

-   **Webots Simulator:** Install Webots from the official Cyberbotics website: [Webots](https://cyberbotics.com/) It is compatible with ROS 2.
-   **ROS 2 (Galactic/Humble or later):** Follow the official ROS 2 installation instructions for your operating system. You can find the documentation here: [ros2 documentation](https://docs.ros.org/en/rolling/index.html) (Replace "rolling" with your ROS 2 distribution, e.g., "galactic", "humble")
-   **webots\_ros2 :** This package provides the necessary interface between ROS 2 and Webots, enabling communication and control of robots within the simulator. You can install the e-puck specific portion of it using:

    ```bash
    sudo apt install ros-<ros2-distro>-webots-ros2
    ```

    Replace `<ros2-distro>` with your ROS 2 distribution (e.g., `galactic`, `humble`).
-   **Colcon (ROS 2 build tool):** If you don't have it, install it:

    ```bash
    sudo apt install python3-colcon-common-extensions
    ```


## Installation

-  **Clone the repository:**

    Navigate to your ROS 2 workspace's `src` directory (or wherever you manage your ROS 2 packages). Then, clone the repository using Git:

    ```bash
    cd <your_ros2_workspace>/src
    git clone <repository_url>
    ```

    Replace `<your_ros2_workspace>` with the path to your ROS 2 workspace (e.g., `~/ros2_ws`) and `<repository_url>` with the URL of this GitHub repository.

-  **Install Dependencies:**

    Navigate back to your ROS 2 workspace's root directory:

    ```bash
    cd <your_ros2_workspace>
    ```

    If your package has any dependencies specified in the `package.xml` file, you can install them using `rosdep`:

    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

    This command will check for any missing dependencies and install them.

-  **Build the package:**

    Use `colcon` to build the package:

    ```bash
    colcon build --symlink-install
    ```

    The `--symlink-install` option creates symbolic links instead of copying files, which speeds up the build process and makes it easier to modify code.

-  **Source the setup file:**

    After building the package, you need to source the setup file to make the ROS 2 executables and libraries available in your current terminal session:

    ```bash
    . install/setup.bash
    ```

    If you want to permanently add the setup file to your environment, you can add the sourcing command to your `.bashrc` or `.zshrc` file:

    ```bash
    echo "source <your_ros2_workspace>/install/setup.bash" >> ~/.bashrc  # For bash
    # or
    echo "source <your_ros2_workspace>/install/setup.bash" >> ~/.zshrc # For zsh
    source ~/.bashrc #or source ~/.zshrc to apply the change to the current terminal
    ```

    Replace `<your_ros2_workspace>` with the actual path to your ROS 2 workspace.


    ## Usage

-  **Launch Webots with the e-puck simulation:**

    Open a new terminal and run the following command to launch the e-puck simulation in Webots:

    ```bash
    ros2 launch epuck_planning robot.launch.py
    ```

    This command will start the Webots simulator with the e-puck robot in your defined world.

-  **Run the Bug 0 algorithm node:**

    Open another new terminal and run the Bug 0 algorithm node:

    ```bash
    ros2 run epuck bug_zero
    ```

    This node will subscribe to the robot's sensor data and publish velocity commands to control the e-puck's movement based on the Bug 0 algorithm.

-  **(Optional) Visualize the robot's path and sensor data:**

    You can use `rqt` or `rviz2` to visualize the robot's path, sensor data, and other relevant information. For example, you can visualize the robot's odometry, sensor readings, and the planned path.

    To start `rqt`, you can run:

    ```bash
    rqt
    ```

    Then add the desired plugins, such as "Plot" for sensor data or "Robot Steering" for manual control.

    To start `rviz2`, you can run:

    ```bash
    rviz2
    ```

    Then configure the plugins to visualize the desired topics.


    ## Contributing and Bug Reporting

If you encounter any bugs, issues, or have suggestions for improvements, please feel free to contribute! You can:

* **Submit a Pull Request:** If you have code changes, bug fixes, or new features, please submit a pull request.
* **Report an Issue:** If you find a bug or have a suggestion, please open an issue on the GitHub repository. Provide as much detail as possible, including steps to reproduce the issue and any relevant error messages.

   

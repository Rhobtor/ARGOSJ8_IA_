

# ARGJ801 Setup Package

## Overview

The `argj801_setup` package is the central configuration and launch management node for a the ROS2 system consisting of multiple nodes. This package is responsible for setting up and managing the lifecycle of various nodes essential for the ARGJ801 mission.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
  - [Launch the System](#launch-the-system)
  - [Nodes](#nodes)
- [Configuration](#configuration)
  - [Parameters](#parameters)
  - [Environment Variables](#environment-variables)
- [Contributing](#contributing)
- [License](#license)
- [Additional Documentation](#additional-documentation)

## Installation

1. Clone the repository:
   ```sh
   git clone git@github.com:Robotics-Mechatronics-UMA/argj801_setup.git
   cd argj801_setup
   ```

2. Install dependencies:
   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```sh
   colcon build
   ```

## Usage

### Launch the System

The main launch file is `J8_launch.py`. This file is responsible for launching the necessary nodes based on the provided parameters. This launch also can be used on both the real and the simulated system. 

#### Parameters

- `robot`: Launch robot-specific nodes if `true` (default: `false`).
- `simulator`: Launch simulator-specific nodes if `true` (default: `false`).
- `use_gui`: Launch the GUI node if `true` (default: `false`).

#### Examples

1. **Launching the Simulator without GUI:**

    ```sh
    ros2 launch argj801_setup main_launch_file.launch.py simulator:=true use_gui:=false
    ```

2. **Launching the Real Robot:**

    ```sh
    ros2 launch argj801_setup main_launch_file.launch.py robot:=true
    ```

3. **Launching the Simulator with GUI:**

    ```sh
    ros2 launch argj801_setup main_launch_file.launch.py simulator:=true use_gui:=true
    ```

By using these parameters, you can easily switch between launching the system for the real robot or the simulator, and optionally include the GUI.
### Nodes

The `argj801_setup` package launchs the following nodes:

- `ctl_mission_node`
- `controller_node`
- `path_following_node`
- `teleoperation_node`
- `path_record_node`
- `ready_node`
- `estop_node`
- `back_home_node`
- `path_manager_node`
- `security_check_node`
- `mpc_node`
- `argj801_ctrl_platform_node`
- `joy_node`
- `android_server_node`
- `laser_segmentation`
- `static_transform_publisher`
- `gui_node`

Each node is configured and launched with specific parameters to perform its designated function in the system.

### Launch and congif files

For more details on configuration and launch files, please refer to their respective README documents:

- [Configuration Files README](./config/README.md)
- [Launch Files README](./launch/README.md)
## Contributing

Contributions are welcome! Please follow the guidelines in [CONTRIBUTING.md](CONTRIBUTING.md) to submit your changes.

## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.

## Additional Documentation

For more detailed information on each package and node, refer to the individual documentation files:

- [ctl_mission](https://github.com/Robotics-Mechatronics-UMA/argj801_ctl_mission.git)
- [path_manager](https://github.com/Robotics-Mechatronics-UMA/path_manager.git)
- [security_check](https://github.com/Robotics-Mechatronics-UMA/security_check.git)
- [argj801_ctl_platform](https://github.com/Robotics-Mechatronics-UMA/argj801_ctl_platform.git)
- [android_ros2_server](https://github.com/Robotics-Mechatronics-UMA/Android_ros2_server_pkg.git)
- [laser_segmentation](git@github.com:Robotics-Mechatronics-UMA/lidar_process_pkgs.git)
- [GUI_pkg](https://github.com/Robotics-Mechatronics-UMA/J8_GUI.git)


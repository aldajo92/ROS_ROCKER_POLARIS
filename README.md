# Polaris GEM e2 Simulation and Path Tracking Controller

This repository contains two main packages for ROS Noetic: `polaris_sim_utils` and `pt_controller`. These packages are designed to work together to provide utilities for the Polaris GEM e2 vehicle and to control its path tracking in a ROS environment.

## Table of Contents

- [Polaris Simulation Utilities](#polaris-simulation-utilities)
- [Path Tracking Controller](#path-tracking-controller)
- [Installation](#installation)
- [Usage](#usage)
- [Design Choices and Architecture](#design-choices-and-architecture)
- [Dockerfile Explanation](#dockerfile-explanation)
- [License](#license)

## Polaris Simulation Utilities

The `polaris_sim_utils` package contains utilities and tools to launch the Polaris GEM e2 vehicle and generate some helper paths to validate the controllers provided by `pt_controller` package.

### Features

- **Utility Scripts**: Scripts to generate some helpful paths.
- **Launch Files**: Pre-configured launch files to execute simulation or just the controller with paths.
- **RVIZ Files**: Custom rviz configuration to validate the path tracking, based on the rviz configuration provided by Polaris GEM e2 vehicle.

## Path Tracking Controller (pt_controller)

The `pt_controller` package contains the implementation of a path tracking controller for the Polaris GEM e2 vehicle. This controller is responsible for following a predefined path using feedback from the vehicle's odometry.

### Features

- **Path Tracking Planner**: Plans the path for the vehicle to follow.
- **Path Tracking Controller**: Controls the vehicle to follow the planned path.

### Architecture and Code Design

The `pt_controller` package is designed with a modular architecture to separate the concerns of path planning and path following. The main components are the `PathTrackingPlanner` and the `PathTrackingController`.

#### PathTrackingPlanner

The `PathTrackingPlanner` is responsible for planning the path that the vehicle should follow. It takes into account the vehicle's current position and the desired path.

**PathTrackingPlanner.h**

```cpp
#ifndef PT_CONTROLLER_PATH_TRACKING_PLANNER_H
#define PT_CONTROLLER_PATH_TRACKING_PLANNER_H

#include "pt_controller/PathTrackingController.h"
#include <ros/ros.h>

class PathTrackingPlanner
{
public:
    enum State
    {
        INIT,
        MISSING_PATH,
        MISSING_ODOM,
        FOLLOW_PATH,
        REACHED_GOAL
    };

    PathTrackingPlanner(double frequency, AbstractPathTrackingController* controller, ros::NodeHandle* nh);
    void execute();
    void stop();
    State getState() const;

private:
    double frequency_;
    AbstractPathTrackingController* controller_;
    ros::NodeHandle* nh_;
    State state_;
};

#endif // PT_CONTROLLER_PATH_TRACKING_PLANNER_H
```


# ROS and Rocker: Polaris #

This repository contains the Dockerfile for the turtlebot3 simulation, using [Rocker](https://github.com/osrf/rocker) (Docker tool) and the docker base image `noetic-desktop-full`.

## Ubuntu 22.04 LTS ##

1. Open a terminal and check if docker is installed:
    ```
    $ docker --version
    ```
    If it is not installed, you can install it as follows:
    ```
    $ sudo apt update && sudo apt install curl
    $ curl -fsSL https://get.docker.com -o get-docker.sh
    $ sudo sh get-docker.sh
    ```
    Add the user to the docker group:
    ```
    $ sudo usermod -aG docker $USER
    $ sudo newgrp docker
    ```

2. Nvidia drivers (Optional): This steps are optional and recomended if you have a nvidia graphics card in your computer/laptop.
    - Open `Additional Drivers` program and install the suggested nvidia driver, as it is shown in the image:
    ![](./.media/nvidia_driver.png)
    - Restart the computer
    - Go to [NVIDIA Container toolkit page](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and follow the steps shown in the image (ignore the optional step to use experimental packages):
    ![](./.media/nvidia_toolkit_install.png)
    - And configure the Nvidia toolkit for docker and containerd:
    ![](./.media/nvidia_toolkit_configuration.png)


## Mac OS (M1/M2 chip) ##

Disclaimer: This process require to run with parallels desktop app which is a paid program, but have a trial with work good for 30 days. I'm not receiving any compensation for this recommendation. I just found it very useful for my work. If you have a better solution, I will be happy to hear it.

1. Install [parallels desktop](https://www.parallels.com/products/desktop/) With Ubuntu:
    - Once parallels is installed in your OS, click on the "+" sign on the top right corner of the window and select "Install Windows or another OS from a DVD or image file".
    - Select Ubuntu with x86_64 emulation.
    ![](.media/parallels_select_ubutu_os.png)
    - Proceed with the installation. Then Click on Download.

    Wait until parallels finishes installing Ubuntu. Once it is done, you should set a password for the user "parallels" and then log in.

2. Validate if docker is installed:
    Parallels comes with docker pre-installed. To validate if it is installed, open a terminal and run the following command:
    ```
    $ docker --version
    ```
    If it is not installed, you can install it as follows:
    ```
    $ sudo apt update && sudo apt install curl
    $ curl -fsSL https://get.docker.com -o get-docker.sh
    $ sudo sh get-docker.sh
    ```
    Add the user to the docker group:
    ```
    $ sudo usermod -aG docker $USER
    $ sudo newgrp docker
    ```

## One you have docker and the Ubuntu Setup ##

3. Configure Rocker:
    - First ensure that the Ubuntu Universe repository is enabled.
        ```
        $ sudo apt update
        $ sudo apt install software-properties-common
        $ sudo add-apt-repository universe
        ```
    - Now add the ROS 2 GPG key with apt.
        ```
        $ sudo apt update && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        ```
    - Then add the repository to your sources list.
        ```
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        ```

    - And install rocker: 
        ```
        $ sudo apt-get install python3-rocker
        ```

4. Get source code:
    - Clone this repository in a specific path:
        ```
        $ git clone https://github.com/aldajo92/ROS_ROCKER_POLARIS.git
        ```
    - Get submodules:
        ```
        $ cd ROS_ROCKER_POLARIS
        $ git submodule update --init --recursive
        ```

5. Build and execute the docker image:
    ```
    $ ./scripts/build
    $ ./scripts/run_cpu # If you don't have nvidia graphics card
    $ ./scripts/run_nvidia # If you have nvidia graphics card configured with docker
    ```

6. Run project:
```
roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
```

## Autor ##
Alejandro Daniel Jose Gómez Flórez - @aldajo92

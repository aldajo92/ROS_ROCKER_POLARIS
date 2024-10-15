# Polaris GEM e2 Simulation and Path Tracking Controller

This repository contains two main packages for ROS Noetic: `polaris_sim_utils` and `pt_controller`. These packages are designed to work together to provide utilities for the Polaris GEM e2 vehicle and to control its path tracking in a ROS environment.

## Polaris Simulation Utilities

The `polaris_sim_utils` package contains utilities and tools to launch the Polaris GEM e2 vehicle and generate some helper paths to validate the controllers provided by the `pt_controller` package.

### Features

- **Utility Scripts**: Scripts to generate helpful paths.
- **Launch Files**: Pre-configured launch files to execute simulation or just the controller with paths.
- **RVIZ Configurations**: Custom RVIZ configuration to validate the path tracking.

## Path Tracking Controller (pt_controller)

The `pt_controller` package contains the implementation of a path-tracking controller for the Polaris GEM e2 vehicle. This controller is responsible for following a predefined path using feedback from the vehicle's odometry.

### Features

- **Path Tracking Planner**: Plans the path for the vehicle to follow.
- **Path Tracking Controller**: Controls the vehicle to follow the planned path by adjusting linear and angular velocities dynamically.

### Architecture and Code Design

The `pt_controller` package is designed with a modular architecture to separate the concerns of path planning and path following. The main components are the `PathTrackingPlanner` and the `PathTrackingController`.

#### PathTrackingController

The `PathTrackingController` is responsible for generating the control actions (linear and angular velocities) that guide the vehicle along the path. It calculates control actions using a feedback control algorithm based on the Pure Pursuit technique.

**Example Code:**

```cpp
std::vector<double> PathTrackingController::computeControlAction(
    const std::vector<double>& position,
    const std::vector<double>& lookahead_point)
{
    if (position.size() != 3)
    {
        ROS_WARN("Position must have 3 elements: x, y, yaw.");
        return {0.0, 0.0};
    }

    double robot_x = position[0];
    double robot_y = position[1];
    double robot_yaw = position[2];

    double lookahead_x = lookahead_point[0];
    double lookahead_y = lookahead_point[1];

    double dx = lookahead_x - robot_x;
    double dy = lookahead_y - robot_y;
    double angle_to_goal = atan2(dy, dx);
    double alpha = angle_to_goal - robot_yaw;

    alpha = atan2(sin(alpha), cos(alpha));

    double kappa = (2 * sin(alpha)) / lookahead_distance_;

    double alignment_factor = cos(alpha);
    double linear_speed = max_linear_speed_ * std::max(alignment_factor, 0.0);
    double angular_speed = linear_speed * kappa;

    return {linear_speed, angular_speed, alpha, kappa};
}
```

### Design Choices and Architecture

- **Modular Design**: The simulation and control logic are separated into distinct packages (`polaris_sim_utils` for simulation utilities and `pt_controller` for path tracking). This makes the system more flexible and scalable for future developments.
  
- **Path Following with Pure Pursuit**: The control strategy for path tracking leverages the Pure Pursuit algorithm, which is simple yet effective for real-time control of autonomous vehicles. By dynamically adjusting the speed based on alignment, the controller ensures smooth and adaptive path following.

### Installation

For detailed installation instructions, please follow the steps provided in this repository: [ROS_Rocker_Turtlebot3SimNoetic](https://github.com/aldajo92/ROS_Rocker_Turtlebot3SimNoetic)

### Usage

**Launching the Simulation and Path Tracking Controller:**
```bash
roslaunch polaris_sim_utils polaris_gazebo.launch
```

### Dockerfile Explanation

The Docker setup allows easy deployment and testing of the project in different environments (Ubuntu and Mac OS). For Nvidia users, the Dockerfile is optimized to leverage GPU-based simulations with the Nvidia container toolkit.

### Author
Alejandro Daniel José Gómez Flórez (aldajo92)

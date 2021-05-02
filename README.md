# Mammoth

## Introduction

The Mammoth repository contains code which will be used on the Mammoth robot. For code that is compatible with Yeti, go to the [Yeti](https://github.com/iscumd/Yeti) repository, or use this code on Yeti/other robots at your own risk. This repository supports launches for both simulation and robot deployments of Mammoth.

## Installing Dependencies

You're going to want to have ROS2 installed. Here's the instructions for [Debian/Ubuntu 20](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

If you are going to be running simulation, follow the Binary Install instructions for [Ignition Gazebo](https://ignitionrobotics.org/docs/edifice/install_ubuntu).


## Building

1. Change directory into your colcon workspace folder. Example: `cd ~/ros_ws/`
2. Clone the repository into your colcon workspace: `vcs import src --input https://raw.githubusercontent.com/iscumd/Mammoth/ros2/mammoth.repos`
3. Build the colcon workspace: `colcon build`
4. Source the local workspace: `. install/setup.bash`

Note: It should be building ros_ign and mammoth packages from source!

## Launching

For information on launching and running the project, head to the following packages and view the ReadMe.
- Robot -> mammoth_snowplow
- Simulation -> mammoth_gazebo

## Development

1. Create a issue describing a bug found or a feature to add.
2. Create a new branch or fork of the project. For branches, the name should describe the feature or bug you are trying to fix. Include a number that tags the issue that the change is associated with. Example: `git checkout -b feat/11-estop-state` or `git checkout -b bug/6-compile-errors` (11 & 6 being the issue number respectively)
3. Make changes on the new branch/fork.
4. Push changes to the branch/fork.
5. Confirm that it builds and solves the issue/implements the feature.
6. Make a pull request to merge the changes to the master branch, and link the appropriate issue to the pull request.
7. Have the pull request reviewed. Make any changes necessary to fix issues found.
8. Merge.

## Documentation

Check the drive for what little we have.

## Fun 
Edit xbox.config.yaml in mammoth_gazebo and increase the values to: 
'''
  teleop_twist_joy_node:
    ros__parameters:
      axis_linear:  # Left thumb stick vertical
        x: 1
      scale_linear:
        x: 1.7
      scale_linear_turbo:
        x: 15.0
  
      axis_angular:  # Left thumb stick horizontal
        yaw: 5
      scale_angular:
        yaw: 10.0
  
      enable_button: 2  # Left trigger button
      enable_turbo_button: 5  # Right trigger button'''
Plug in an xbox controller and launch the file    

# Mammoth

## Introduction

The Mammoth repository contains code which will be used on the Mammoth robot. For code that is compatible with Yeti, go to the [Yeti](https://github.com/iscumd/Yeti) repository, or use this code on Yeti/other robots at your own risk. This repository supports launches for both simulation and robot deployments of Mammoth.

## Building

1. Change directory into your catkin workspace source folder. Example: `cd ~/catkin_ws/src`
2. Clone the repository into your catkin workspace: `git clone https://github.com/iscumd/Mammoth.git --recurse-submodules`
3. Change directory back to catkin workspace: `cd ..`
4. Build the catkin workspace: `catkin_make` (hint: Don't forget to source your workspace)

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

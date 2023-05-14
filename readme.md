# TUM-Panda-HOI Assistive Robot Project
## This project is still under developing
## General Infomation
1. System: Ubuntu 20.04
2. Kernel version: 5.9.1-rt20
3. ROS distribution: Noetic
4. Camera: Intel Realsense Depth Camera D435i
5. Robot: Franka Emika Panda

## Pre-request
Please setup libfranka and realtim kernel first, you can follow the instruction [here](https://frankaemika.github.io/docs/installation_linux.html).

Create a workspace for this project, open a terminal, run:
```
mkdir -p ws_project/src
cd ~/ws_project
```
Clone the packages of franka_ros into the src folder:
```
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
```
install any missing dependencies:
```
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
```
Once you are done with installation, you can continue to [download MoveIt source](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html), remember to put all the packages in the ws_project/src folder, you should stop at "Build your Catkin Workspace" section, for we will build the workspace later, please also note that there is no need to download panda_moveit_config package and the example code provided by MoveIt, because we will use our own code.

## Download Source Code
Before you start with this section, please make sure that the "panda_moveit_config" package is deleted, which could be downloaded in previous steps, we will use our own panda_moveit_config package.

You can download the code for this project as follows:
```
cd ~/ws_project
# TBD, will be added once the work is done
```
it is important to also put these packages in the same workspace as before.

Then, build the workspace (this may take a while):
```
cd ~/ws_project
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```
## Launch Files
1. You can create a customized .world file for Gazebo simulation by running:
   ```
   roslaunch gazebo_sim edit_world.launch
   ```
## Some Demos (TBD)
Herem are some demos that you can run to get an overview of this project.
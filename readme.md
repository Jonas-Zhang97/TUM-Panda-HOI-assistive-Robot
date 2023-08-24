# TUM-Panda-HOI Assistive Robot Project (developing)

This repository hosts the code for the project of research internship with [LMT](https://www.ce.cit.tum.de/lmt/startseite/).

Supervisor: [Yuankai Wu, M.Sc](https://www.ce.cit.tum.de/lmt/team/mitarbeiter/wu-yuankai/).

## Main Functionalities

With the project's codebase, objects positioned on a table plane are isolated as outliers through the implementation of the RanSaC algorithm. Subsequently, a depth image that exclusively encompasses the objects is generated. With objects depth image, bounding boxes provided by YOLO are used to select a desired target and generate a depth image with only the target object. The resulting image serves as the input for the contact graspnet, generating potential grasp candidates.

Following the generation of these grasp poses, the pose possessing the utmost score is selected. Employing the capabilities of MoveIt, the robot is empowered to execute a pick-and-place sequence, seamlessly transferring the object to a human recipient.

## General Infomation

1. System: Ubuntu 20.04
2. Kernel version: 5.9.1-rt20
3. ROS distribution: [Noetic](http://wiki.ros.org/noetic/Installation)
4. Camera: Intel Realsense Depth Camera D435i
5. Robot: Franka Emika Panda
6. GPU: NVidia GTX 1080Ti
7. Driver Version: 525.125.06
8. CUDA Version: 12.0

## Pre-request

For the control of the Franka Emika Panda robot arm, a real-time kernel is essential. However, due to its incompatibility with Nvidia driver and CUDA, a dual-computer setup becomes necessary. One computer is designated for robot control, while the other handles Pose detection.

Both computers must have ROS (Robot Operating System) installed. Ensure that both computers are connected to the same network and capable of seamless communication, following the guidelines outlined in [this tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines). The computer for controling the robot should work as the master node.

Now, proceed with the configuration of the computers using the following steps:

### On the Computer to Control the Robot

1. The Franka_ros package and the realtime kernel have to be configured. Please refer to the [official tutorial](https://frankaemika.github.io/docs/installation_linux.html) provided by Franka Emika for detailed instuctions.

2. MoveIt! packages (or at least panda_moveit_config) are required for the robot control, see installition instuctions [here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).

### On the Computer to Perform the Calculations

Firstly, install and configure the [Nvidia driver](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html) and install [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

Create a workspace for the codebase:

```bash
mkdir -p ~/ws_hoi/src
cd ~/ws_hoi/src 
```
  
in the src folder, clone and build the [darknet_ros](https://github.com/leggedrobotics/darknet_ros). Because the MoveIt! is required for the path planning, it is recommanded to configure the MoveIt! libraries in this workspace.

## Configure and Build the Packages

clone this repo in the workspace:

```bash
git clone https://github.com/Jonas-Zhang97/TUM-Panda-HOI-assistive-Robot.git
```

Then, nevigate to `TUM-Panda-HOI-assistive-Robot/vision/vision_common`, configure the parameters to suit your implementaion (make sure the directories exist).

Build the workspace as follows:

```bash
cd ~/ws_hoi
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Run the Code

Firstly, launch the realsense camera:

```bash
roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true align_depth:=true
```

On the computer to control the robot:

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=<fci_ip>
```

On the computer to perform calculations, open 3 terminals:

```bash
roslaunch object_detection object_detection.launch
```

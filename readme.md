# TUM-Panda-HOI Assistive Robot Project

This repository hosts the code for the project of research internship with [LMT](https://www.ce.cit.tum.de/lmt/startseite/).

Supervisor: [Yuankai Wu, M.Sc](https://www.ce.cit.tum.de/lmt/team/mitarbeiter/wu-yuankai/).

## Main Functionalities

With the project's codebase, objects positioned on a table plane are isolated as outliers through the implementation of the RanSaC algorithm. Subsequently, a depth image that exclusively encompasses the objects is generated. With objects depth image, bounding boxes provided by YOLO are used to select a desired target and generate a depth image with only the target object. The resulting image serves as the input for the contact graspnet, generating potential grasp candidates.

Following the generation of these grasp poses, the pose possessing the utmost score is selected. Employing the capabilities of MoveIt, the robot is empowered to execute a pick-and-place sequence, seamlessly transferring the object to a human recipient.

## General Infomation

1. System: Ubuntu 20.04
2. Kernel version: 5.9.1-rt20
3. ROS distribution: Noetic
4. Camera: Intel Realsense Depth Camera D435i
5. Robot: Franka Emika Panda
6. GPU: NVidia GTX 1080Ti
7. Driver Version: 525.125.06
8. CUDA Version: 12.0

## Pre-request

For the control of the Franka Emika Panda robot arm, a real-time kernel is essential. However, due to its incompatibility with Nvidia driver and CUDA, a dual-computer setup becomes necessary. One computer is designated for robot control, while the other handles grasp pose detection and path planning.

Both computers must have [ROS](http://wiki.ros.org/noetic/Installation) (Robot Operating System) installed. Ensure that both computers are connected to the same network and capable of seamless communication, following the guidelines outlined in [this tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines). The computer for controling the robot should work as the master node.

Now, proceed with the configuration of the computers using the following steps:

### On the Computer to Control the Robot

1. Connect robot to the computer;

2. The Franka_ros package and the realtime kernel have to be configured. Please refer to the [official tutorial](https://frankaemika.github.io/docs/installation_linux.html) provided by Franka Emika for detailed instuctions;

3. MoveIt! packages (or at least panda_moveit_config) are required for the robot control, see installition instuctions [here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html).

### On the Computer to Perform the Calculations

Connect your camera to the computer.

Subsequently, install the [Nvidia driver](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html) and [CUDA](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

Clone and configure the [Contact GraspNet](https://github.com/Jonas-Zhang97/contact_graspnet_hoi) repo, this repo is forked from [Contact GraspNet](https://github.com/NVlabs/contact_graspnet) with modified hyperparameters to suit the project's implementation, specifically, to corpe with the situation when the size of input point cloud is limited.

Create a workspace for the codebase:

```bash
mkdir -p ~/ws_hoi/src
cd ~/ws_hoi/src 
```
  
Inside the `src` directory, begin by cloning and building the [darknet_ros](https://github.com/leggedrobotics/darknet_ros) package. Given that MoveIt! is essential for effective path planning, MoveIt! libraries should be configured within this workspace as well.

## Configure and Build the Packages

Clone this repo in the workspace:

```bash
git clone https://github.com/Jonas-Zhang97/TUM-Panda-HOI-assistive-Robot.git
```

Next, proceed to the file labeled `TUM-Panda-HOI-assistive-Robot/vision/vision_common/config/settings.yaml`. Here, adjust the parameters to align with the specifics of your implementation, please ensure the corresponding directories are in place. Similarly modify the subscribed topic of the `object_detection` package. Navigate to `TUM-Panda-HOI-assistive-Robot/vision/object_detection/launch/object_detection.launch` and edit the "image" argument to match the RGB image topic of your camera.

As previously instructed, access the file labeled `TUM-Panda-HOI-assistive-Robot/motion/motion_common/config/path_settings.yaml`. In this location, tailor the paths to match your implementation's requirements.

Subsequently, open `TUM-Panda-HOI-assistive-Robot/common/launch/camera_pose.launch`, adjust the launch file of tf-transformation-broadcaster to the one you created during camera calibration, this guarantees that the camera pose is correctly configured and that robot can move correctly. In the same file, also modify the launch file for camera to meet your implementation.

Finally, build the workspace:

```bash
cd ~/ws_hoi
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Run the Code

On the computer to connect and control the robot, launch the robot moveit control:

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=<fci_ip>
```

On the computer for calculations, open 3 terminals. Firstly, launch the object_detection, which launches camera and pose broadcaster in the mean time:

```bash
roslaunch object_detection object_detection.launch
```

in the second one, launch npy generator:

```bash
roslaunch vision_common cloud2npy.launch
```

in the last one, launch the motion control nodes:

```bash
roslaunch motion_common pick_place.launch
```

## Publish Command

To generate a .npy file of the desired object for the contact_graspnet, publish the name of the object to `/hoi/target_object_name` as follows:

```bash
rostopic pub -1 /hoi/target_object_name /std_msgs/String "data: '<name>'"
```

The .npy file will be saved in the directory that you set previously.

Then, run the inference for the object you specified:

```bash
python /path/to/contact_graspnet/contact_graspnet/inference.py --np_path=/path/to/npy/file/<name>.npy
```

After the result is generated, you can publish the object's name to `/hoi/target_object` to perform pick-and-place task:

```bash
rostopic pub -1 /hoi/target_object /std_msgs/String "data: '<name>'"
```

## Topics

### Published Topics

The topics published by this project's codebase are following:

1. /hoi/objects_cloud: The objects point cloud (sensor_msgs/PointCloud2) given by plane segmentation
2. /hoi/image_from_cloud: The depth image (sensor_msgs/Image), which contains all the objects given by plane segmentation
3. /hoi/depth_image: The depth image (sensor_msgs/Image), which contains only the target object
4. /hoi/grasp_pose: The selected grasp pose (geometry_msgs/PoseStamped) with highest score

### Subscribed Topics

1. hoi/target_object_name: Command (std_msgs/String) to generate a .npy file
2. hoi/target_object: Command (std_msgs/String) to pick and place the specified object

## Drawbacks

1. The project is operating within the ROS framework; however, a notable setback lies in the absence of a ROS interface for the contact_graspnet component, moreover, the contact graspnet should be ran in a specific conda environment, which makes it more independent. This deficiency is contributing to a less-than-smooth pipeline, causing interruptions in the expected workflow.

2. The presence of noise in the camera's data is significantly impacting the generation of high-quality, and in some cases, valid grasp candidates. This challenge is comprehensively documented in the file labeled `TUM-Panda-HOI-assistive-Robot/vision/doc/test_record.md`. One potential approach to enhance performance involves object segmentation through RGB image analysis, which might yield improved outcomes.

3. The current perspective of the camera is yielding inconsistent and inaccurate detection results when utilizing the darknet model. Addressing this issue necessitates an intricate process of fine-tuning, wherein adjustments are made to optimize the detection performance and reliability.

The mentioned limitations and challenges will be actively refined in the future works.

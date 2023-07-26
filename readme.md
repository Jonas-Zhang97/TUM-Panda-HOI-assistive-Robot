# TUM-Panda-HOI Assistive Robot Project

## This project is still under developing

## 1. General Infomation

1. System: Ubuntu 20.04
2. Kernel version: 5.9.1-rt20
3. ROS distribution: Noetic
4. Camera: Intel Realsense Depth Camera D435i
5. Robot: Franka Emika Panda

## 2. Pre-request

To set up your system with [libfranka](https://frankaemika.github.io/docs/libfranka.html), [franka_ros](https://frankaemika.github.io/docs/franka_ros.html), and [MoveIt](https://moveit.ros.org/), follow the steps below:

1. Begin by installing libfranka and franka_ros:

   - Refer to the [libfranka installation guide](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) and follow the instructions to build from source.
   - Similarly, follow the [franka_ros installation guide](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages) to build the ROS packages. Building from source is recommended.
   - After installing libfranka and franka_ros, set up the realtime kernel as described in the [Franka Emika documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). This step ensures real-time capabilities for controlling the Franka Emika robot.

2. Finally, install MoveIt. To do this:

   - Visit the [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/index.html) page.
   - Follow the provided [installation instructions](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) to set up MoveIt on your system.
   - During the installation process, you can skip the panda_moveit_config package since the modified package will be provided in my project.
   - Additionally, the moveit_tutorial package is not required, so there is no need to install it.

## 3. Download Source Code

Before you start with this section, please make sure that the "panda_moveit_config" package is deleted, which could be downloaded in previous steps, we will use our own panda_moveit_config package.

You can download the code for this project into the MoveIt workspace as follows:

```bash
cd path/to/moveit_ws/src
git clone https://github.com/Jonas-Zhang97/TUM-Panda-HOI-assistive-Robot.git
```

it is important to also put these packages in the same workspace as before.

Compile the workspace again, then you are ready to play some demos.

## 4. Start demo

If you have a robot and want to perform our functionalities on it, go to the Franka Emika Desk, enable FCI and launch:

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=10.162.83.121
```

replace the robot_ip with the ip of your robot.

If you don't have access to any Panda arm, you can also simulate it in your custormized gazebo world.

## 5. Some Demos (TBD)

Here are some demos that you can run to get an overview of this project.

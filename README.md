# ROS Lidar Robot

This project aims to implement SLAM using the RPLIDAR on a real robot with ROS. The ultimate goal is for the robot to autonomously navigate in a room. This project is being worked on in collaboration with Byron Ho (byronho).

### Project Status

The robot can now be manually controlled with ROS from a keyboard to drive around a room and gather lidar data. The next step is autonomouos navigation via SLAM.

![](https://drive.google.com/uc?export=view&id=1dbUq8MTbeKoB9IzGMe3JaV-vG-qusDUl) 

### Robot Hardware

The final robot hardware configurationcan be seen in the pictures below. The robot is driven by 4 stepper motors connected to a base. The lidar is mounted to a lidar mount plate attached to the base. A large battery bank is mounted under the robot base. This battery supplies power to RPi, Arduino, and the RPLidar. The Lipo battery under the lidar supplies power to the stepper motors.

Side          |  Top | Iso
:-------------------------:|:-------------------------:|:-------------------------:
![](https://i.imgur.com/B815roU.jpg) | ![](https://i.imgur.com/MukmtIb.jpg) | ![](https://i.imgur.com/Axxuxbe.jpg)

## Instructions

The ROS master is running on a VM on laptop. Commands are sent from the VM (by pressing keys on keyboard) to the RPi onboard the robot. The RPi then sends commands to the Arduino (which acts as a ROS node) to control the motors. The RPLIDAR is connected to the RPi and is sending lidar data back to the VM for processing.

### Creating Catkin Workspace

In both VM and RPi:
  1) Create a `catkin_ws` folder in VM and RPi. This is your catkin workspace.
  2) Create a `src` folder in `catkin_ws`
  3) Clone this repo to `/catkin_ws/src` by running `git clone https://github.com/akhild555/lidar_robot_ROS.git`
  4) Navigate to the `/catkin_ws` folder and build by running `catkin_make`
  5) Source the setup file by running `source /devel/setup.bash` in `/catkin_ws`
  
### ROS Network Settings

In order to get the VM to communicate with the RPi, we need to configure ROS network settings.

In VM (ROS Master):
  1) Run `hostname -I` to get IP Address
  2) Run `export ROS_IP=<IP Address>` (Fill in `<IP Address>` with the IP Address from step 1.
  3) Run `export ROS_MASTER_URI=http://<IP Address>:11311` (Fill in `<IP Address>` with the IP Address from step 1.
  4) You should add the commands from step 2 and 3 to the end of your `~/.bashrc` file to avoid running them everytime you open a new terminal.
  
In RPi:

  5) Run `hostname -I` to get IP Address
  6) Run `export ROS_IP=<IP Address>` (Fill in `<IP Address>` with the IP Address from step 5.
  7) Run `export ROS_MASTER_URI=http://<IP Address>:11311` (Fill in `<IP Address>` with the IP Address from step 1 (so the master's IP Address!!!).
  8) You should add the commands from step 6 and 7 to the end of your `~/.bashrc` file to avoid running them everytime you open a new terminal.
  
### Keyboard Control of Robot

First, upload `keyboard_ctrl.ino` file in the `/Arduino` folder into the Arduino.

In VM (ROS Master) first:

  1) Start ROS Master by running `roscore`. Make sure the ROS network settings are configured as described above!
  2) In another terminal, run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`. This will start the ROS node that will allow you to control the robot via keyboard commands. If you don't have the teleop_twist_keyboard package, you can install it by running `sudo apt-get install ros-noetic-teleop-twist-keyboard`. 

In RPi:

  3) Run `rosrun rosserial_python serial_node.py`. This will create a node for the Arduino to communicate with other nodes on ROS. If rosserial is not installed, run  `sudo apt-get install ros-<ros_distro>-rosserial-arduino`.

You can now go back to the terminal in VM with teleop_twist_keyboard running and control the robot with the `i` key for forward translation, `u` for counterclockwise rotation, `o` for clockwise rotation, and `.` for backward translation.
  
### Lidar Data

To view RPLIDAR data:

In RPi:

  1) In a new terminal, navigate to `/catkin_ws` and source the setup file by running `source /devel/setup.bash`
  2) Run `rosrun rplidar_ros rplidarNode`

In VM:

  3) In a new terminal, navigate to `/catkin_ws` and source the setup file by running `source /devel/setup.bash`
  4) Run `rosrun tf static_transform_publisher 0 0 0 0 0 0 0 laser_frame laser`
  5) In another terminal, navigate to `/catkin_ws` and source the setup file. Then, run `roslaunch rplidar_ros view_rplidar.launch`
  6) Step 5 will open RVIZ. In the side bar on the left, change "Fixed Frame" to `laser_frame`.
  
You should now see the lidar data the RPLIDAR is capturing.

Autonomous navigation to come...

## Acknowledgements 

Some aspects of the robot hardware (primarily the mecanum wheels) are taken from Dejan's project here `https://howtomechatronics.com/projects/arduino-mecanum-wheels-robot/#unlock`. The robot base and RPLIDAR mount are unique to this robot.
The `/hector_slam` and `/rplidar_ros` folders in this repo were taken from NickL77's repo found here: `https://github.com/NickL77/RPLidar_Hector_SLAM`
  

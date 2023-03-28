# Anti-drone
This repository contains the path planning algorithm for an anti-drone system. 

## Usage instructions
### Tested with Ubuntu 20.04, ROS Noetic
Create the catkin_ws, clone the repo, build and source the workspace using
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/AshwinDisa/anti_drone.git
$ cd ..
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## SITL
Install and build
PX4 Firmware - https://github.com/PX4/PX4-Autopilot or Ardupilot - https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

mavros - https://docs.px4.io/main/en/ros/mavros_installation.html

QGC - https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

Extra dependencies
```
pip3 install modern-robotics
pip3 install future
```

Add the following lines to your bashrc or copy paste in terminal everytime you want to launch px4 sitl in gazebo environment 
```
$ cd /<path-to-firmware>/PX4-Autopilot
$ source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```
Make sure this line is present in your bashrc
```
$ export ROS_MASTER_URI=http://127.0.0.1:11311/
```

Launch the gazebo environment, mavros and open the QGC application
```
$ roslaunch px4 mavros_posix_sitl.launch
```

Takeoff the drone using 'commander takeoff' command in terminal or directly from QGC

Run any of the rosbags with the command and remap the rostopic names
```
$ cd catkin_ws/src/anti_drone/src/rosbags/
$ rosbag play -l circle.bag /mavros/local_position/pose:=/mavros/local_position/pose/old /mavros/local_position/velocity_local:=/mavros/local_position/velocity_local/old
```

Make the visualization code executable and run 
```
$ cd catkin_ws/src/anti_drone/src
$ chmod +x visualization_3.py
$ python3 visualization_3.py
```

Finally run the anti-drone algorithm
```
$ cd catkin_ws/src/anti_drone/src
$ chmod +x anti_v1.py
$ python3 anti_v1.py
```
Change the flight mode to "OFFBOARD" for PX4 Firmware and "GUIDED" for Ardupilot

## Hardware
### Tested with Ubuntu 20.04, ROS Noetic on Raspberry Pi 4 Model B, CubeOrange and Ardupilot v...
Install packages and dependencies on Rpi as needed
```
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
pip3 install modern-robotics
pip3 install MAVProxy
pip3 install pymavlink
```
Run the following commands in seperate terminals to connect to the fcu and gcs 
```
mavproxy.py --out=udp:127.0.0.1:14551
roslaunch mavros apm.launch fcu_url:=udp://:14551@ gcs_url:=udp://:14550@<gcs-ip-address>:14550
```
Run any of the rosbags with the command and remap the rostopic names
```
$ cd catkin_ws/src/anti_drone/src/rosbags/
$ rosbag play -l straight.bag /mavros/local_position/pose:=/mavros/local_position/pose/old /mavros/local_position/velocity_local:=/mavros/local_position/velocity_local/old
```
Make the visualization code executable and run 
```
$ cd catkin_ws/src/anti_drone/src
$ chmod +x visualization_3.py
$ python3 visualization_3.py
```

Finally run the anti-drone algorithm 
```
$ cd catkin_ws/src/anti_drone/src
$ chmod +x anti_v1.py
$ python3 anti_v1.py
```
Change the flight mode to "OFFBOARD" for PX4 Firmware and "GUIDED" for Ardupilot



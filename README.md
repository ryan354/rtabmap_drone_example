# rtabmap_drone_example
2D navigation example of a drone using [move_base](http://wiki.ros.org/move_base) with [mavros](http://wiki.ros.org/mavros)/[px4](https://github.com/PX4/PX4-Autopilot) and [rtabmap](wiki.ros.org/rtabmap_ros) visual SLAM using Jetson Xavier NX

Overview video (click to watch on Youtube):

[![Youtube](https://i.imgur.com/UKLtD7L.gif)](https://youtu.be/A487ybS7E4E)

## Dependencies

Tested on ROS Melodic and ROS Noetic with the corresponding PX4 versions below.

```bash
sudo apt install \
   ros-$ROS_DISTRO-joy \
   ros-$ROS_DISTRO-imu-complementary-filter \
   ros-$ROS_DISTRO-teleop-twist-joy \
   ros-$ROS_DISTRO-geographic-msgs \
   ros-$ROS_DISTRO-dwa-local-planner \
   libgeographic-dev \
   geographiclib-tools \
   libgstreamer1.0-dev
sudo pip3 install numpy toml packaging jinja2 empy numpy
# and gstreamer (https://github.com/PX4/PX4-Autopilot/issues/13117):
sudo apt-get install libgstreamer-plugins-base1.0-dev
   
# If rtabmap is not already built from source:
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
sudo apt install ros-$ROS_DISTRO-rtabmap


```

### Mavros
```bash

cd ~/catkin_ws/src
# (With mavros master branch there are a lot of "Detected jump back in time" TF errors)
git clone https://github.com/mavlink/mavros.git && cd mavros && git checkout 1.9.0 && cd ..
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
cd ~/catkin_ws
catkin_make
```

## Usage

```

roslaunch rtabmap_drone_example slam.launch
roslaunch rtabmap_drone_example rviz.launch
# Arm and take off:
rosrun rtabmap_drone_example offboard
```
 * Manual control: If a joystick is plugged, you can send twists by holding L1 and moving the joysticks. Hold L1+L2 with left joystick down to land (be gentle to land smoothly), then hold left joystick in bottom-right position to disarm after the drone is on the ground.
 * Autonomous control: use "2D Nav Goal" button in RVIZ to set a goal to reach 

![](https://raw.githubusercontent.com/matlabbe/rtabmap_drone_example/master/doc/example.jpg)

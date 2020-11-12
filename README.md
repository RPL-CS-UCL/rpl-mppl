# rpl-mppl
Description of the UCL's RPL MPPL robot

The code has been tested in ROS Melodic

## Step 1: install dependencies
```bash
sudo apt-get install python-rosdep ros-melodic-eigen-conversions ros-melodic-kdl-parser ros-melodic-effort-controllers ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-combined-robot-hw ros-melodic-joint-state-controller ros-melodic-velocity-controllers ros-melodic-twist-mux ros-melodic-diff-drive-controller ros-melodic-costmap-prohibition-layer ros-melodic-moveit ros-melodic-moveit-core ros-melodic-teb-local-planner ros-melodic-move-base ros-melodic-moveit-kinematics ros-melodic-robot-localization ros-melodic-combined-robot-hw ros-melodic-joint-limits-interface ros-melodic-gmapping ros-melodic-amcl ros-melodic-position-controllers ros-melodic-joint-trajectory-controller ros-melodic-moveit-visual-tools ros-melodic-moveit-ros-planning-interface ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-global-planner ros-melodic-joint-state-publisher-gui ros-melodic-moveit-visual-tools ros-melodic-rqt-joint-trajectory-controller ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-joint-trajectory-controller ros-melodic-*controller* ros-melodic-rospy* ros-melodic-rosmsg 

sudo apt-get install -y rosbash
```
Note that in melodic this package is also needed: ros-melodic-moveit-ros-occupancy-map-monitor

## Step 2: download the required packages in the workspace
```bash
mkdir -p rpl-mppl/src
cd rpl-mppl/src
```
Clone this repository and it's submodules
```
git clone --recurse-submodules https://github.com/rpl-as-ucl/rpl-mppl.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build -j 1
```
It may also need: git clone https://github.com/rpl-as-ucl/gazebo_ros_pkgs.git

## Step 3-1: run the rbkairos with moveit
Terminal 1:
```bash
source devel/setup.bash
roslaunch rbkairos_sim_bringup rbkairos_complete.launch
```
Terminal 2:
```bash
source devel/setup.bash
roslaunch rbkairos_moveit_config rbkairos_planning_execution.launch
```

## Step 3-1: run the mppl with moveit
Terminal 1:
```bash
source devel/setup.bash
roslaunch mppl_sim_bringup mppl_complete_odom.launch
```

## Step 3-2: run the franka with moveit
Terminal 1:
```bash
source devel/setup.bash
roslaunch panda_simulator simulation.launch
```
## To update any github REPO that is forked
```bash
$ cd github-services
$ git remote add upstream git://github.com/RobotnikAutomation/github-services.git
$ git fetch upstream
# then: (like "git pull" which is fetch + merge)
$ git merge upstream/master master
# or, better, replay your local work on top of the fetched branch
# like a "git pull --rebase"
$ git rebase upstream/master
```

## Bugs:
```bash
sudo apt-get install python-empy
sudo apt-get install libuvc-dev 
sudo apt-get install ros-melodic-hector-nav-msgs
sudo apt-get install ros-melodic-hector-trajectory-server
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-cob-navigation-config
```

# Realsense Installation on Ubuntu 18.04LTS, ROS Melodic
Follow the steps:
1. https://github.com/IntelRealSense/realsense-ros
```bash
export ROS_VER=Melodic
sudo apt-get install ros-$ROS_VER-realsense2-camera
sudo apt-get install ros-$ROS_VER-realsense2-description
roslaunch realsense2_camera rs_camera.launch
```

2. In case of errors (): https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#prerequisites
```bash
sudo update
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
modinfo uvcvideo | grep "version:"
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev 
sudo apt-get install libglfw3-dev
pip install pyrealsense2
```

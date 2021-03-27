# rpl-mppl
Description of the UCL's RPL MPPL robot

The code has been tested in ROS Kinetic, Melodic

## Step 1: install dependencies
```bash
sudo apt-get install python-rosdep ros-kinetic-eigen-conversions ros-kinetic-kdl-parser ros-kinetic-effort-controllers ros-kinetic-controller-manager ros-kinetic-transmission-interface ros-kinetic-combined-robot-hw ros-kinetic-joint-state-controller ros-kinetic-velocity-controllers ros-kinetic-twist-mux ros-kinetic-diff-drive-controller ros-kinetic-costmap-prohibition-layer ros-kinetic-moveit ros-kinetic-moveit-core ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-moveit-kinematics ros-kinetic-robot-localization ros-kinetic-combined-robot-hw ros-kinetic-joint-limits-interface ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-position-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-moveit-visual-tools ros-kinetic-moveit-ros-planning-interface ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-global-planner ros-kinetic-joint-state-publisher-gui ros-kinetic-moveit-visual-tools ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-gazebo-ros-control ros-kinetic-joint-trajectory-controller ros-kinetic-*controller* ros-kinetic-rospy* ros-kinetic-rosmsg 

sudo apt-get install -y rosbash
```
Note that in melodic this package is also needed: ros-kinetic-moveit-ros-occupancy-map-monitor
You may also need to update to gazebo 9: sudo apt-get install gazebo9* libgazebo9* ros-kinetic-gazebo9-*

## Step 2: download the required packages in the workspace
```bash
mkdir -p rpl-mppl/src
cd rpl-mppl/src
git clone https://github.com/rpl-as-ucl/egh_gripper_common.git
git clone https://github.com/rpl-as-ucl/egh_gripper_controller.git
git clone https://github.com/rpl-as-ucl/franka_ros.git
git clone https://github.com/rpl-as-ucl/hector_gazebo.git
git clone https://github.com/rpl-as-ucl/mppl_common.git
git clone https://github.com/rpl-as-ucl/mppl_odom_broadcaster.git
git clone https://github.com/rpl-as-ucl/mppl_sim.git
git clone https://github.com/rpl-as-ucl/panda_moveit_config.git
git clone https://github.com/rpl-as-ucl/panda_simulator.git
git clone https://github.com/rpl-as-ucl/rbkairos_common.git
git clone https://github.com/rpl-as-ucl/rbkairos_sim.git
git clone https://github.com/rpl-as-ucl/rcomponent.git
git clone https://github.com/rpl-as-ucl/realsense_gazebo_plugin.git
git clone https://github.com/rpl-as-ucl/robotnik_elevator_interface.git
git clone https://github.com/rpl-as-ucl/robotnik_gazebo_models.git
git clone https://github.com/rpl-as-ucl/robotnik_gazebo_plugins.git
git clone https://github.com/rpl-as-ucl/ros_astra_camera.git
git clone https://github.com/rpl-as-ucl/robotnik_msgs.git
git clone https://github.com/rpl-as-ucl/robotnik_sensors.git
git clone https://github.com/rpl-as-ucl/rpl_panda_with_rs.git
git clone https://github.com/rpl-as-ucl/scripts.git
git clone https://github.com/rpl-as-ucl/summit_xl_common.git
git clone https://github.com/rpl-as-ucl/summit_xl_sim.git
git clone https://github.com/rpl-as-ucl/summit_xls_ur5_common.git
git clone https://github.com/rpl-as-ucl/universal_robot.git
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
sudo apt-get install ros-kinetic-hector-nav-msgs
sudo apt-get install ros-kinetic-hector-trajectory-server
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-cob-navigation-config
```

# Realsense Installation on Ubuntu 16.04LTS, ROS Kinetic
Follow the steps:
1. https://github.com/IntelRealSense/realsense-ros
```bash
export ROS_VER=kinetic
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

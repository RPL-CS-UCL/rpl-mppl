# rpl-mppl
Description of the UCL's RPL MPPL robot

The code has been tested in ROS Kinetic

## Step 1: install dependencies
```bash
  sudo apt-get install ros-kinetic-eigen-conversions ros-kinetic-kdl-parser ros-kinetic-effort-controllers ros-kinetic-controller-manager ros-kinetic-transmission-interface ros-kinetic-gazebo-ros-pkgs ros-kinetic-combined-robot-hw ros-joint-state-controller ros-kinetic-velocity-controllers ros-kinetic-twist-mux ros-kinetic-diff-drive-controller ros-kinetic-costmap-prohibition-layer ros-kinetic-moveit ros-kinetic-moveit-core ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-moveit-kinematics ros-kinetic-robot-localization ros-kinetic-combined-robot-hw ros-kinetic-joint-limits-interface ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-position-controllers ros-kinetic-joint-trajectory-controller ros-kinetic-moveit-visual-tools ros-kinetic-moveit-ros-planning-interface ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-global-planner ros-kinetic-joint-state-publisher-gui ros-kinetic-gazebo-ros-control
```

## Step 2: download the required packages in the workspace
```bash
mkdir -p rpl-mppl/src
cd rpl-mppl/src
```

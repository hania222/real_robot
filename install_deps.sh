#!/bin/bash
set -e

echo "Installing ROS 2 Jazzy dependencies..."

sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-navfn-planner \
  ros-jazzy-dwb-core \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-rplidar-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  python3-pip

pip3 install websockets

echo "then:  colcon build --symlink-install"

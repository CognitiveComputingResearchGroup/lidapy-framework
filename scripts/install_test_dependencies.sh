#! /bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update -qq
sudo apt-get install ros-jade-ros-base
sudo rosdep init
rosdep -y update
sudo apt-get install gazebo5
sudo apt-get install ros-jade-gazebo-ros-pkgs
pip install nose
pip install pyyaml
pip install rospkg
sudo apt-get install -y python-rosinstall
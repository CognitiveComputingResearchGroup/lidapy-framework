#! /bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update -qq
sudo apt-get install ros-jade-ros-base
sudo rosdep init
rosdep -y update
sudo apt-get install ros-jade-gazebo-ros-pkgs

LIDAPY_FRAMEWORK=$(find $HOME -type d -name lidapy-framework 2>/dev/null)

# Assumes script is being run from the scripts directory
if [ -e "$LIDAPY_FRAMEWORK/requirements.txt" ]; then
    pip install -r "$LIDAPY_FRAMEWORK/requirements.txt"
fi

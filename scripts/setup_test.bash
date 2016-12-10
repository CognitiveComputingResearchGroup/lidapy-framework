# Setup ROS environment variables
source /opt/ros/jade/setup.bash

LIDAPY_FRAMEWORK=$(find $HOME -type d -name lidapy-framework 2>/dev/null)

export PYTHONPATH=$PYTHONPATH:$LIDAPY_FRAMEWORK/src

# lidapy-framework
[![Build Status](https://travis-ci.org/CognitiveComputingResearchGroup/lidapy-framework.svg?branch=master)](https://travis-ci.org/CognitiveComputingResearchGroup/lidapy-framework)

A Python-based LIDA framework using [ROS](http://wiki.ros.org/ROS/Tutorials).

## Dependencies

[ROS Jade](http://wiki.ros.org/jade) is a dependency of the lidapy-framework, and is only supported on the following 
[Ubuntu](http://www.ubuntu.com) releases:
* Trusty (14.04)
* Utopic (14.10)
* Vivid (15.04)

## Setup Options
 
Two setup options are provided below for installing the lidapy-framework and the necessary dependencies.  

**Option 1** requires a supported version of the Ubuntu OS, and requires manual installation of the lidapy-framework
dependencies.  
**Option 2** uses the [common-ccrgdev](https://github.com/CognitiveComputingResearchGroup/common-ccrgdev) development
environment.  This option will work for non-supported OSes (using [VirtualBox](http://www.virtualbox.org)) and provides 
automated dependency installation (using [Vagrant](https://www.vagrantup.com/) provisioning).

If you plan on using Gazebo (3D environment simulation/visualization) for your agent's environment, option 1 is 
highly encouraged (as option 2 can result in unacceptably slow performance).

### Option 1 - Setup for Supported Ubuntu OS 


#### ROS (Jade)
Follow the [Ros Jade Installation Instructions](http://wiki.ros.org/jade/Installation/Ubuntu)

#### Gazebo (Gazebo5)
While Gazebo is not required, several of the lidapy-framework examples depend on gazebo5.

Once ROS has been installed, Gazebo can be installed using the following commands:

~~~
sudo apt-get install gazebo5
sudo apt-get install ros-jade-gazebo-ros-pkgs
~~~

#### Get lidapy-framework
To setup lidapy you need to download or clone this repository. To download click 
[here](https://github.com/CognitiveComputingResearchGroup/lidapy-framework/archive/master.zip) and unzip the downloaded 
file or to clone into a directory where you want lidapy-framework to live run the following from that directory:

~~~
git clone https://github.com/CognitiveComputingResearchGroup/lidapy-framework.git
~~~
  
### Option 2 - Setting up a virtual machine

If you do not want to go through all these steps you may want to using our common CCRG Develpoment environment. You can 
get this by going the [common-ccrgdev](https://github.com/CognitiveComputingResearchGroup/common-ccrgdev) repository.

## Create a ROS workspace for lidapy-framework
A script is provided that will create and configure a ROS workspace for developing lidapy-framework agents. 
Run this script by executing the following command from the lidapy-framework directory
    
Usage ./scripts/setup.sh [full-path-of-workspace-directory]
    
Example:
~~~  
./scripts/setup.sh /home/user/lidapy-workspace
~~~

When running the script the first time, you will have to run ```source ~/.lidapy/setup.bash```. The setup.sh script will 
automatically update your bash environment configuration file (i.e, .bashrc) to execute setup.bash for all subsequent 
bash sessions.

## Run an agent

NOTE: Before you use the following command to run an agent make sure that the package is in the $ROSPATH or the 
directory pointed to by the `roscd` command.

~~~
roslaunch [package-name] [launch-file]
~~~

Example:

~~~      
roslaunch talker_listener agent.launch
~~~   

## Create an agent

To create your own lidapy-framework based agents run:

~~~
catkin_create_pkg [agent-name] lidapy-rosdeps [other-ros-package-dependencies]
~~~
Example:
~~~
catkin_create_pkg text-attractor lidapy-rosdeps
~~~     

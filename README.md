# lidapy-framework
A proof of concept for a distributed Python-based LIDA framework using ROS

## Setup lidapy

### Prerequisites

Setting up existing machine:
#### ROS
To install ROS run (on Ubuntu 14.x+): 

      sudo apt-get install ros-jade-ros-base
      
### Get lidapy
  To setup lidapy you need to download or clone this repository. To download click [here](https://github.com/CognitiveComputingResearchGroup/lidapy-framework/archive/master.zip) and unzip the downloaded file or to clone into a directory where you want lidapy-framework to live run the following from that directory:
    
      git clone https://github.com/CognitiveComputingResearchGroup/lidapy-framework.git
  
### Setup lidapy workspace
  We provide a script that will setup the lidapy workspace for you and configure the environment variables as needed. Run this script by executing the following command from lidapy-framework directory
    
Usage ./scripts/setup.sh [full-path-of-workspace-directory]
    
Example:
  
    ./scripts/setup.sh /home/user/lidapy-workspace
   
When running the script the first time, you will have to run ```source ~/.lidapy/setup.bash```. The setup.sh script will automatically update your bash environment configuration file (i.e, .bashrc) to execute setup.bash for all subsequent bash sessions.
    
## Run an agent

   roslaunch [package-name] [launch-file]
   
   Example:
      
        roslaunch talker_listener agent.launch
      
## Create an agent

To create your own lidapy-framework based agents run:

catkin_create_pkg [agent-name] lidapy-rosdeps [other-ros-package-dependencies]

Example:

      catkin_create_pkg text-attractor lidapy-rosdeps
     

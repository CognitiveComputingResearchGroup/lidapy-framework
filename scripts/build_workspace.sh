#! /bin/bash

display_usage()
{
    echo -e ""
    echo -e "Create a ROS workspace with lidapy dependencies."
    echo -e ""
    echo -e "Usage:"
    echo -e "  $0 <directory>"  
    echo -e ""
    echo -e "Options:"
    echo -e "     -h     displays this help message"
    echo -e "     -n     non-interactive mode"
    echo -e "     -v     verbose output"

}

display_yn_prompt()
{
    if [[ $interactive -eq 0 ]]; then
        return
    fi

    while true; do
        echo "$1"
        echo "Is this correct? [YN]"
        read choice
        
        case $choice in 
            [Yy]) choice="Y"; break;;
            [Nn]) choice="N"; break;;
            *) ;;
        esac
    done
}

# Options variables
verbose=0
interactive=1

# Process command line arguments
OPTIND=1 # Reset getopts
while getopts "hnv" opt; do
    case "$opt" in
    h)
        display_usage
        exit 1
        ;;
    n) 
        interactive=0
        ;;
    v) 
        verbose=1
        ;;
    esac
done

shift $((OPTIND-1))
[ "$1" = "--" ] && shift

# Set target location for new ROS workspace
workspace_dir=$1
if [[ -z $workspace_dir ]]; then
    display_usage 
    exit 1
fi

if [[ ! "$workspace_dir" = /* ]]; then
    workspace_dir="$PWD/$workspace_dir"
fi

display_yn_prompt "Workspace directory = $workspace_dir" 
if [[ $choice == "N" ]]; then
    echo "Aborting."
    exit 1
fi

if [[ -e $workspace_dir ]]; then
    echo "Aborting. Workspace directory ($workspace_dir) already exists!"
    exit 1
fi

# Attempt to auto-detect the location of the lidapy-framework
lidapy_framework_dir=$(find $HOME -type d -name lidapy-framework 2>/dev/null)
while [[ -z $lidapy_framework_dir ]]; do

    # Unable to locate the lidapy-framework; if interactive mode then prompt user for location
    # else abort with error
    if [[ $interactive -eq 0 ]]; then
        echo "Aborting. Unable to find lidapy-framework directory!"
        exit 1
    fi

    echo "Please supply the location of the lidapy-framework directory."
    read lidapy_framework_dir
done

display_yn_prompt "LidaPy Framework = $lidapy_framework_dir" 
if [[ $choice == "N" ]]; then
    echo "Aborting."
    exit 1
fi

if [[ ! -d $lidapy_framework_dir ]]; then
    echo "Aborting. LidaPy Framework directory does not exist or is unreadable!"
    exit 1
fi

mkdir -vp $workspace_dir/src
cp -R $lidapy_framework_dir/ros/lidapy_rosdeps $workspace_dir/src
cp -R $lidapy_framework_dir/examples/* $workspace_dir/src

cd $workspace_dir

display_yn_prompt "Would you like to build the examples workspace?"
if [[ $choice == "Y" || $interactive -eq 0 ]]; then
    catkin_make
fi

mkdir $HOME/.lidapy

if [[ -e "$HOME/.lidapy/setup.bash" ]]; then
    display_yn_prompt "A lidapy-framework configuration file already exists.  Override?"
    if [[ $choice == "Y" || $interactive -eq 0 ]]; then
        rm -v "$HOME/.lidapy/setup.bash"
    fi
fi

echo -e "\nsource $workspace_dir/devel/setup.bash" >> $HOME/.lidapy/setup.bash
echo -e "\nexport PYTHONPATH=\$PYTHONPATH:$lidapy_framework_dir/src" >> $HOME/.lidapy/setup.bash

display_yn_prompt "Update BASH configuration for lidapy-framework?"
if [[ $choice == "Y" || $interactive -eq 0 ]]; then
    echo "Adding \"source \$HOME/.lidapy/setup.bash\" to .bashrc."
    echo -e "\nsource \$HOME/.lidapy/setup.bash" >> $HOME/.bashrc
fi

echo "Execute \"source \$HOME/.lidapy/setup.bash\" to update environment for lidapy framework."

exit 0

#! /bin/bash

display_usage()
{
    echo -e ""
    echo -e "Create a ROS workspace with lidapy dependencies."
    echo -e ""
    echo -e "Usage:"
    echo -e "  $0 <directory>"  
    echo -e ""
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
interactive=0

# Process command line arguments
OPTIND=1 # Reset getopts
while getopts "hiv" opt; do
    case "$opt" in
    h)
	display_usage
	exit 1
	;;
    i) 
	interactive=1
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

display_yn_prompt "Add devel/setup.bash to .bashrc?"
if [[ $choice == "Y" || $interactive -eq 0 ]]; then
    echo -e "\nsource $workspace_dir/devel/setup.bash" >> $HOME/.bashrc
fi

display_yn_prompt "Update .bashrc with PYTHONPATH for lidapy-framework?"
if [[ $choice == "Y" || $interactive -eq 0 ]]; then
    echo -e "\nexport PYTHONPATH=\$PYTHONPATH:$lidapy_framework_dir/src" >> $HOME/.bashrc
fi

echo "Success!  Please restart your terminal for changes to take effect."

exit 0

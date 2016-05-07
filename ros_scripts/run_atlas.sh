echo "starting roscore..."

roscore &

echo "initializing camera output..."

# multiplex ros topics for debugging purposes (so we can see camera output)
rosrun topic_tools mux /stereo/left/image_raw /multisense_sl/left/image_raw &
rosrun topic_tools mux /stereo/right/image_raw /multisense_sl/right/image_raw &
rosrun topic_tools mux /stereo/left/camera_info /multisense_sl/left/camera_info &
rosrun topic_tools mux /stereo/right/camera_info /multisense_sl/right/camera_info &
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc &

# show camera output for debugging
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color &

sleep 1

echo "starting Gazebo world... Please be patient. Only run 'agentstarter.py' when Gazebo has finished loading"

# launch actual ROS world; use VRC cheats (for "fake walking")
# see http://gazebosim.org/tutorials?tut=drcsim_fakewalking&cat=drcsim
VRC_CHEATS_ENABLED=1 roslaunch drcsim_gazebo vrc_task_3.launch debug:=true 
# custom world in folder "testworld"
# VRC_CHEATS_ENABLED=1 roslaunch testworld mound.launch debug:=true

echo "finished. closing image windows."

# after we are done, kill helper topic multiplexers
killall mux
killall stereo_image_proc

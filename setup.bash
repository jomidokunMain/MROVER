#!/bin/bash

####
## Assumption: you're at the 'oscar' directory.
source ~/.bashrc
source /home/jesudara/rosBash/noeotic/runSource.sh  
source catkin_ws/devel/setup.bash
source /home/jesudara/bimi_publish/MROVER/mavros_ws/devel/setup.bash
conda activate mrover



# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot > /dev/null 2>&1
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo > /dev/null 2>&1

##
# add neural_net folder to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)/neural_net
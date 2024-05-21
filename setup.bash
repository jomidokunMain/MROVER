#!/bin/bash

####
## Assumption: you're at the 'oscar' directory.
source ~/.bashrc
source /opt/ros/foxy/setup.bash 
source dev_ws/install/setup.bash



##
# add neural_net folder to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)/neural_net
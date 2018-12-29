#!/usr/bin/env bash

echo $@
START_POINT=$1
SIM_ID=$2
PROPERTY_PATH=$3

cd /home/docker/ROS/workspace

source devel/setup.bash

roslaunch rossimulator rossim.launch &

cd /home/docker

nohup java -jar SimKernel.jar ${SIM_ID} ${PROPERTY_PATH} &
java -jar Core.jar ${START_POINT} ${PROPERTY_PATH}
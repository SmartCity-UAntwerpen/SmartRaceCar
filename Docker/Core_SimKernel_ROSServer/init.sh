#!/usr/bin/env bash

set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"


sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install build-essential -y
gcc -v
make -v

sudo apt-get install ros-indigo-navigation -y

cd /home/docker/ROS/WS_Race
catkin_make
source devel/setup.bash

cd /home/docker/ROS/WS_Nav
catkin_make

source devel/setup.bash

cd /home/docker/ROS/workspace/src

catkin_init_workspace

cd ..

catkin_make

source devel/setup.bash
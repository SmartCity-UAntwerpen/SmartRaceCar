# SmartRacecar - Ros-Nodes

The low level modules that will handle all low level functions of the smartracecar vehicle category on the ROS operating system.

## Modules
* ROS-Node: The vehicle runs on ROS with all it's cores and nodes. There are nodes for LIDAR, navigation, driving, route tracking, ... There is therefore also a node to communicate all this data and instructions to the high-level Core module. This is called the CoreLinker node.
* ROS-Server: A deployed ROS-Server is used to make the ROS calculations for all simulated vehicles (SimKernel). This way they can mimic a real vehicle relativly realistic. In the files it's called the rossimulator node.


## Use Guide

### Mapping

1) launching the roscore

2) launching mapping.launch from workspace launchcar 
```roslaunch launchcar mapping.launch```
In order to drive the car using the keyboard, add following lines to the launch file
```
<node name="keyboard" type="keyboard.py" pkg="race" output="screen" launch-prefix="xterm -e"/>
<node name="talker" type="talker.py" pkg="race" output="screen"/>
```

3) via VNC: 
```roslaunch hector_slam_launch custom.launch ```
→ is used to start recording the map, rviz will open

4) ```rosrun map_server map_saver -f nameForMap ```
→ run this command after the run is done BEFORE closing the hector_slam_launch. Insert a name of the map on nameForMap

5) Tranfser.pgm to local host to edit the incorrect black lines with GIMP
→ bv; 
```scp ubuntu@172.16.0.2:~/Git/MAP2017/ROS/WS_Nav/src/f1tenth_2dnav/maps/kelder.pgm ~/Downloads/```

### Driving
in ```~/Git/SmartRacecar/ROS/``` run ```/workspace/scripts/launchCar -i be.pool.ntp.org -l LOCATION``` with ```LOCATION``` the waypoint where the car is located

SAFETY PRECAUTIONS:<br/>
change speed: in ```~/Git/SmartRacecar/ROS/workspace/src/corelinker/scripts/javalinker.py``` change ```navstack_speed``` (1.5 is minimum to be able to drive)<br/>

make sure your can kill ```navigation.launch``` and ```move_base.launch``` processes

have a ```keyboard.py``` module running to make sure you can stop the car if necessary

## Developed by

Jansen Wouter,
Jens de Hoog

University of Antwerp - 2017

# SmartRacecar - Ros-Nodes

The low level modules that will handle all low level functions of the smartracecar vehicle category on the ROS operating system.

## Modules
* ROS-Node: The vehicle runs on ROS with all it's cores and nodes. There are nodes for LIDAR, navigation, driving, route tracking, ... There is therefore also a node to communicate all this data and instructions to the high-level Core module. This is called the CoreLinker node.
* ROS-Server: A deployed ROS-Server is used to make the ROS calculations for all simulated vehicles (SimKernel). This way they can mimic a real vehicle relativly realistic. In the files it's called the rossimulator node.


## Developed by

Jansen Wouter,
Jens de Hoog

University of Antwerp - 2017

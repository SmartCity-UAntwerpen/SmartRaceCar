SmartRacecar - Java modules
================
The high level modules that will handle all high level functions of the smartracecar vehicle category.

## Modules
 - Core: Java-module that runs on the racecar itself. In charge of passing on messages to the vehicles ROS nodes. Communicates with the Manager module to register, receive route job requests and handles these to give instructions to the ROS nodes.
 - Manager : Java-module that runs on the backend servers. Registers all racecars and manages them. Receives job requests from backend SmartCity and processes these to the vehicles. Also deals with the simulated vehicles.


## How to use
 - Build and constructed as Intelij IDEA project with Maven.
 - Compiled on Java 1.8.
 - Requires maps folder with all available offline maps and a maps.xml file for both Core and Manager.
 - Requires waypoints folder with all available waypoints in a waypoints.xml for Manager.
 - Has two build .jar's for the Core and Manager modules in release folder. Build with Maven.
 - Requires startup parameter of the startlocation (number ID of waypoint) for the Core, and startup parameter mapname for the Manager.

## Developed by

Jansen Wouter,
Jens de Hoog

University of Antwerp - 2017

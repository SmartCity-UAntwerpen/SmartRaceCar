SmartRacecar - Java-Core
================
The high level module that will send control messages to the underlying ROS system of the RaceCar. This module will communicate with the RacecarManager systems as well.

## Functionality
 - Connects to vehicle over Socket connection. Connects to RacecarManager over MQTT and REST interfaces. 
 - Handles initial vehicle startup to set the correct offline map parametersn, get all waypoint information and register the vehicle.
 - Handles route requests
 
## How to use
 - Build and constructed as Intelij IDEA project with Maven.
 - Compiled on Java 1.8.
 - Requires maps folder with all available offline maps and a maps.xml file.
 - Requires startup parameter of the startlocation (number ID).

## Developed by

Jansen Wouter,
Jens de Hoog

University of Antwerp - 2017

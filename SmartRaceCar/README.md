SmartRacecar - Java modules
================
The high level modules that will handle all high level functions of the smartracecar vehicle category.

## Modules
 - Core: Java-module that runs on the racecar itself. In charge of passing on messages to the vehicles ROS nodes. Communicates with the Manager module to register, receive route job requests and handles these to give instructions to the ROS nodes.
 - Manager : Java-module that runs on the backend servers. Registers all racecars and manages them. Receives job requests from backend SmartCity and processes these to the vehicles. 
 - SimDeployer: Management of simulation vehicles. Receives tasks of SimWorker module to create, edit and stop new simulated RaceCars. Dynamically deploys the simulated vehicles.
 - SimKernel: The simulated version of a ROS-Node in Java. To simulate multiple vehicles a simulated variant of the ROS-Node was made. It acts as if it was a real driving ROS vehicle. It communicates the exact same way to a Core as a ROS-Node would. To realistically mimic a ROS self-driving vehicle it communicates with a ROS-Server to make it do the ROS-based calculations. 

## How to use
 - Build and constructed as IntelliJ IDEA project with Maven.
 - Compiled on Java 1.8.
 - Requires maps folder with all available offline maps and a maps.xml(found in ROS folder) for the Core and a folder with all available offline maps for the Manager (found in release folder).
 - Has 4 build .jar's for the Core and Manager modules in release folder. Build with Maven. 
 - Has external properties files found in the release folder for when Jar's are used outside of Intelij. (When used in IntelliJ it will default to hardcoded parameters defined in the class).
 - Requires startup parameter of the startlocation (number ID of waypoint) and listener(server) and sending(client) ports for the Core, and listener(server) and sending(client) ports for the Simkernel.
 - JavaDoc is included in the JavaDoc folder to explain all classes,variables and methods in detail.
 
## Developed by

Jansen Wouter,
Jens de Hoog

University of Antwerp - 2017

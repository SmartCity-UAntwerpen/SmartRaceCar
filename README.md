<p align="center"> 
<img src="http://i.imgur.com/fB1RHbZ.png">
</p>

# SmartRacecar
Project for the SmartCity-project developing all systems for the F1 RaceCar that is self-driving using offline maps and LIDAR.

## Infrastructure
The SmartCar consists of a set of Java-Modules which contain the management, simulation of a RaceCar and high-level Core modules of the RaceCar. The RaceCar itself is a ROS operated vehicle. 

## Modules
* RacecarBackend: Management of all RaceCars. Registers new vehicle, receives and dispatches jobs and request from the SmartCity to indvidual vehicles and keeps track off all active RaceCars both real and simulated. 
* SimDeployer: Management of simulation vehicles. Receives tasks of SimWorker module to create, edit and stop new simulated RaceCars. Dynamically deploys the simulated vehicles.
* Core: The high-level element of a RaceCar. Handles the initial setup of the vehicle and registers with the Manager. Is in constant communication to receive data from the car itself or instructions from the Manager that it will translate to be execute to the car. 
* ROS-Node: The vehicle runs on ROS with all it's cores and nodes. There are nodes for LIDAR, navigation, driving, route tracking, ... There is therefore also a node to communicate all this data and instructions to the high-level Core module. This is called the CoreLinker node.
* SimKernel: The simulated version of a ROS-Node in Java. To simulate multiple vehicles a simulated variant of the ROS-Node was made. It acts as if it was a real driving ROS vehicle. It communicates the exact same way to a Core as a ROS-Node would. To realistically mimic a ROS self-driving vehicle it communicates with a ROS-Server to make it do the ROS-based calculations. 
* ROS-Server: A deployed ROS-Server is used to make the ROS calculations for all simulated vehicles (SimKernel). This way they can mimic a real vehicle relativly realistic. In the files it's called the rossimulator node.
## What is in this GIT
* A IntelliJ IDEA project for all Java modules (RacecarBackend,SimDeployer, SimKernel and Core). Using Maven for dependencies and building. All runconfigurations and configuration files are included. 
* ROS nodes for the JavaLinker and ROS-Server.

## Developed by

Jansen Wouter,
Jens de Hoog

## Extended by

Robrecht Daems

University of Antwerp - 2017

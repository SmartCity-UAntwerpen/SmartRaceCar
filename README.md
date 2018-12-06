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

## REST interface (smartcity.ddns.net:8081/carmanager)
* register/{startWaypoint}: 
  + called by vehicle core to register itself in the backend 
  + returns the id of the new vehicle in plain text.
* posAll: 
  + get all vehicle positions
  + returns JSON-formatted positions:
    ```
    [
      - {
          idVehicle: 901,
          idStart: 49,
          idEnd: 49,
          percentage: 100
        },
      - {
          idVehicle: 901,
          idStart: 49,
          idEnd: 49,
          percentage: 100
        }
    ]
    ```
 * getVehicles:
    + get all vehicles
    + returns a JSON-formatted list of all vehicles
      ```
      [
        - 901: 
           {
              ID: 901,
            - location: 
              {
                  idVehicle:  901,
                  idStart: 49,
                  idEnd: 49,
                  percentage: 100
              }
              occupied: false,
              available: true,
              heartbeat: "Jan 15, 2018 12:21:16 PM"
           },
          - 903: 
            {
              ID: 903,
            - location: 
              {
                  idVehicle:  903,
                  idStart: 46,
                  idEnd: 46,
                  percentage: 100
              }
              occupied: false,
              available: true,
              heartbeat: "Jan 15, 2018 12:21:20 PM"
           }
      ]
      ```
  * calcWeight/{idStart}/{idStop}
    + get cost calculation for all available vehicles between two waypoints
    + return JSON-formatted costs for all vehicles
      ```
        [
          - {
                status: false,
                weightToStart: 20,
                weight: 4,
                idVehicle: 901
            },
          - {
                status: false,
                weightToStart: 0,
                weight: 4,
                idVehicle: 903
            }
       ]
      ```
   * getMapName
     + get the name of the currently used map
     + return plain text formatted name
   * delete/{id}
      + called to delete a vehicle with a certain ID
      + returns a status OK if successful
   * getMapPGM/{mapName}
      + download a map's PGM file
      + return octet-stream of the file
   * getMapYAML/{mapName}
      + download a map's YAML file
      + return octet-stream of the file
   * executeJob/{idJob}/{idVehicle}/{idStart}/{idEnd}
      + make the vehicle with 'idVehicle' drive from 'idStart' to 'idEnd'
      + called by backbone => only call when racecar is in a safe position to move (not on a table)
      + return plain text status
          

## What is in this GIT
* A IntelliJ IDEA project for all Java modules (RacecarBackend,SimDeployer, SimKernel and Core). Using Maven for dependencies and building. All runconfigurations and configuration files are included. 
* ROS nodes for the JavaLinker and ROS-Server.

## Developed by

Jansen Wouter,
Jens de Hoog

## Extended by

Robrecht Daems

University of Antwerp - 2017

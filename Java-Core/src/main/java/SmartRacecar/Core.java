package SmartRacecar;

import org.json.simple.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;

interface eventListener {
    void locationUpdate(float x,float y);
    void jobRequest(int[] wayPoints);
    void updateRoute();
}

public class Core implements eventListener {

    //Hardcoded Settings
    String mqttBrokerUrl = "tcp://broker.hivemq.com:1883";
    String mqttUsername = "username";
    String mqttPassword = "password";
    int clientPort = 5005;
    int serverPort = 5006;

    MQTTUtils mqttUtils;
    TCPUtils tcpUtils;

    int ID; // ID given by SmartCity Core
    ArrayList<Map> loadedMaps = new ArrayList<>(); // list of all loaded maps
    HashMap <Integer, WayPoint> wayPoints = new HashMap<>();
    Map currentMap = null; // current map being used in the city
    int passengers = 0; // amount of passengers inside //TODO implement systems for passengers
    Position currentLocation = new Position(0,0); // most recent position of the vehicle
    Queue<Integer> currentRoute = new LinkedList<>();// all waypoints to be handled in the current route

    double speed = 0;
    double rotation = 0;

    public Core() throws InterruptedException {
        register();
        loadedMaps = Map.loadMapsFromFolder("maps");
        requestMap();
        requestWaypoints();
        mqttUtils = new MQTTUtils(ID,mqttBrokerUrl,mqttUsername,mqttPassword,this);
        tcpUtils = new TCPUtils(clientPort,serverPort,this);
        tcpUtils.start();

        while (true) {
            Thread.sleep(1000);
//            rotation = 20;
//            sendWheelStates();
//            Thread.sleep(1000);
//            rotation = -20;
//            sendWheelStates();
//            Thread.sleep(1000);
//            rotation = 0;
//            speed = 1;
//            sendWheelStates();
//            Thread.sleep(1000);
//            speed = 2;
//            sendWheelStates();
//            Thread.sleep(1000);
//            speed = 0;
//            sendWheelStates();
//            Thread.sleep(1000);
        }
    }

    public void updateRoute(){
        if(!currentRoute.isEmpty()){
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            JSONObject parentData = new JSONObject();
            JSONObject childData = new JSONObject();
            childData.put("x", nextWayPoint.getX());
            childData.put("y", nextWayPoint.getY());
            parentData.put("nextWayPoint", childData);
            tcpUtils.sendUpdate(JSONUtils.JSONtoString(parentData));
        }else{
            routeCompleted();
        }
    }

    public void routeCompleted(){
        System.out.println("[Core] [DEBUG] Route completed");
        //TODO send message to backbone to complete route
    }

    public static void main(String[] args) throws IOException, InterruptedException {
       new Core();
    }

    public void register(){
        //TODO add rest call to get ID
        ID = 0;
    }

    public void sendWheelStates() {
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("throttle", (int) speed);
        childData.put("steer", (int) rotation);
        parentData.put("drive", childData);
        tcpUtils.sendUpdate(JSONUtils.JSONtoString(parentData));
    }



    public void exitCore(){
        tcpUtils.run = false;
        tcpUtils.closeTCP();
        mqttUtils.closeMQTT();
        System.exit(0);
    }

    public void locationUpdate(float x,float y) {
        System.out.println("[CORE] [DEBUG] LOCATION UPDATED");
        currentLocation.setPosition(x,y);
        mqttUtils.publishMessage(ID + "/location", x + "," + y);
    }

    public void requestMap(){
        //TODO request map name through REST
        String mapname = "V314";
        boolean found = false;
        for (Map map : loadedMaps) {
            if(map.getName().equals(mapname)){
                System.out.println("[Core] [DEBUG] Map '" + mapname + "' found. Setting as current map.");
                currentMap = map;
                found = true;
            }
            if(found)break;
        }
        if(!found){
            //TODO download map from backbone
            System.out.println("[Core] [DEBUG] Map '" + mapname + "' not found. Downloading...");
            loadedMaps.add(Map.addMap("test",0,0, (float) 0.25));
        }
    }

    public void requestWaypoints(){
        //TODO request waypoints through REST
        WayPoint A = new WayPoint(1,0,0,1);
        WayPoint B = new WayPoint(2,1,0,1);
        WayPoint C = new WayPoint(3,0,1,1);
        WayPoint D = new WayPoint(4,1,1,1);
        wayPoints.put(A.getID(),A);
        wayPoints.put(B.getID(),B);
        wayPoints.put(C.getID(),C);
        wayPoints.put(D.getID(),D);
        System.out.println("[Core] [DEBUG] All waypoints received.");
    }

    @Override
    public void jobRequest(int[] wayPointIDs) {
        if(currentRoute.isEmpty()){
            for (int wayPointID : wayPointIDs) {
                if(wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    System.out.println("[Core] [DEBUG] Added waypoint " + wayPointID + " at " + wayPoints.get(wayPointID).getX() + "," + wayPoints.get(wayPointID).getY() + ".");
                }else{
                    System.err.println("[Core] [ERROR] Waypoint with ID '" + wayPointID + "' not found.");
                }

            }
            updateRoute();

        }else{
            System.err.println("[Core] [ERROR] Current Route not completed. Not adding waypoints.");
        }
    }
}
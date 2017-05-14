package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.Level;

//interface to trigger certain events from other objects such as TCP Sockets or MQTT.


public class Core implements TCPListener, MQTTListener {

    //Hardcoded elements.
    private boolean debugWithoutRos = true; // debug parameter to stop attempts to send over sockets when ROS-Node is active.
    private Log log;
    private Level level = Level.INFO; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final String restURL = "http://localhost:8080/carmanager";
    private final String SocketAdress = "localhost";
    private final int serverPort = 5005;
    private final int clientPort = 5006;
    private final String mapFolder = "maps";

    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    private long ID; // ID given by RaceCarManager.
    private HashMap<String, Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private HashMap<Integer, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.
    private Queue<Integer> currentRoute = new LinkedList<>();// All waypoint IDs to be handled in the current route.
    private int routeSize = 0; // Current route's size.
    private boolean connected = false; // To verify socket connection to vehicle.
    private static int startPoint; // Starting position on map. Given by main argument.
    private boolean occupied = false; // To verify if racecar is currently occupied by a route job.

    public Core() throws InterruptedException, IOException {
        log = new Log(this.getClass(), level);
        restUtils = new RESTUtils(restURL);
        requestWaypoints();
        register();
        mqttUtils = new MQTTUtils(ID, mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/" + ID + "/job");
        tcpUtils = new TCPUtils(SocketAdress,serverPort, clientPort, this);
        tcpUtils.start();

        if (!debugWithoutRos) {
            connectSend();
            while (!connected) {
                Thread.sleep(1);
            }
        } else {
            connected = true;
        }
        sendStartPoint();
        loadedMaps = XMLUtils.loadMaps(mapFolder);
        requestMap();
    }

    //Send connection check over sockets
    private void connectSend() {
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
    }

    //Event to be called when connection to car has been made
    public void connectReceive() {
        connected = true;
        Log.logInfo("CORE", "Connected to car.");

    }

    //Register vehicle with RaceCarManager
    private void register() {
        HashMap<String, String> queryParams = new HashMap<>();
        queryParams.put("startwaypoint", Integer.toString(startPoint));
        String id = restUtils.getTextPlain("register", queryParams);
        ID = Long.parseLong(id, 10);
        Log.logInfo("CORE", "Vehicle received ID " + ID + ".");
    }

    //Request all possible waypoints from RaceCarManager
    private void requestWaypoints() {
        Type typeOfHashMap = new TypeToken<HashMap<Integer, WayPoint>>() {
        }.getType();
        wayPoints = (HashMap<Integer, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
        assert wayPoints != null;
        for (WayPoint wayPoint : wayPoints.values()) {
            Log.logConfig("CORE", "Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
        }
        Log.logInfo("CORE", "All possible waypoints(" + wayPoints.size() + ") received.");
    }

    //Set the starting point for the vehicle. Sending it over the socket connection.
    private void sendStartPoint() {
        Log.logInfo("CORE", "Starting point set as waypoint with ID " + startPoint + ".");
        if (!debugWithoutRos)
            tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startPoint", wayPoints.get(startPoint)));
    }


    //REST call to RaceCarManager to request the name of the current map. If this map is not found in the offline available maps, it does another
    //REST download call to download the map and store it in it's /mapFolder and add it to the maps.xml file.
    //After that it sends this information to the vehicle to be used over the socket connection.
    private void requestMap() throws IOException {
        String mapName = restUtils.getTextPlain("getmapname");
        //String mapName = "zbuilding";
        if (loadedMaps.containsKey(mapName)) {
            Log.logInfo("CORE", "Current map '" + mapName + "' found.");
            sendCurrentMap(mapName);
        } else {
            Log.logConfig("CORE", "Current map '" + mapName + "' not found. Downloading...");
            restUtils.getFile("getmappgm/" + mapName, "maps", mapName, "pgm");
            restUtils.getFile("getmapyaml/" + mapName, "maps", mapName, "yaml");
            addMap(mapName);
            Log.logInfo("CORE", "Current map '" + mapName + "' downloaded.");
            sendCurrentMap(mapName);
        }
    }

    //Add a new map to the /mapFolder folder and update the maps.xml file.
    private void addMap(String name) {
        Map map = new Map(name);
        try {
            DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

            Document document = documentBuilder.parse(mapFolder + "/maps.xml");
            Element root = document.getDocumentElement();

            Element newMap = document.createElement("map");

            Element newName = document.createElement("name");
            newName.appendChild(document.createTextNode(name));
            newMap.appendChild(newName);

            root.appendChild(newMap);

            DOMSource source = new DOMSource(document);

            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
            StreamResult result = new StreamResult(mapFolder + "/maps.xml");
            transformer.transform(source, result);

        } catch (ParserConfigurationException | SAXException | IOException | TransformerException e) {
            Log.logSevere("CORE", "Could not add map to XML of maps." + e);
        }
        loadedMaps.put(name, map);
        Log.logConfig("CORE", "Added downloaded map : " + name + ".");

    }

    //Send the name and other information of the current map to the vehicle over the socket connection.
    private void sendCurrentMap(String mapName) {
        if (!debugWithoutRos) tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentMap", loadedMaps.get(mapName)));

    }

    //Tto be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
    //Sends information of the next waypoint over the socket connection to the vehicle.
    private void updateRoute() {
        if (!currentRoute.isEmpty()) {
            mqttUtils.publishMessage("racecar/" + ID + "/waypoint", String.valueOf(currentRoute.peek()));
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            if (!debugWithoutRos) tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("nextWayPoint", nextWayPoint));
            Log.logInfo("CORE", "Sending next waypoint with ID " + nextWayPoint.getID() + " (" + (routeSize - currentRoute.size()) + "/" + routeSize + ")");
            if(debugWithoutRos){
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                wayPointReached();
            }

        } else {
            routeCompleted();
        }
    }

    //Event call over interface to be used when socket connection received message that waypoint has been reached.
    public void wayPointReached() {
        Log.logInfo("CORE", "Waypoint reached.");
        updateRoute();
    }

    //When all waypoints have been completed the vehicle becomes available again.
    private void routeCompleted() {
        Log.logInfo("CORE", "Route Completed.");
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "done");
    }

    private void routeError() {
        Log.logWarning("CORE", "Route error. Route Cancelled");
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "error");
    }

    private void routeNotComplete() {
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "notcomplete");
    }

    //Send wheel states to the vehicle over the socket connection. useful for emergency stops and other requests.
    private void sendWheelStates(float throttle, float steer) {
        if (!debugWithoutRos) tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("drive", new Drive(steer, throttle)));
        Log.logInfo("CORE", "Sending wheel state Throttle:" + throttle + ", Steer:" + steer + ".");
    }

    //Event call over interface for when the socket connection receives location update. Publishes this to the RaceCarManager over MQTT.
    public void locationUpdate(Point location) {
        Log.logInfo("CORE", "Location Updated.");
        mqttUtils.publishMessage("racecar/" + ID + "/location", JSONUtils.objectToJSONString(location));
    }

    public void parseMQTT(String topic, String message) {
            if (!occupied) {
                String[] waypointStringValues = message.split(" ");
                try {
                    int[] waypointValues = new int[waypointStringValues.length];
                    for (int index = 0; index < waypointStringValues.length; index++) {

                        waypointValues[index] = Integer.parseInt(waypointStringValues[index]);
                    }
                    jobRequest(waypointValues);
                } catch (NumberFormatException e) {
                    Log.logWarning("CORE", "Parsing MQTT gives bad result: " + e);
                }
            } else {
                Log.logWarning("CORE", "Current Route not completed. Not adding waypoints.");
                routeNotComplete();
            }
    }

    public void parseTCP(String message) {
        if (JSONUtils.isJSONValid(message)) {
            //parses keyword to do the correct function call.
            switch (JSONUtils.getFirst(message)) {
                case "location":
                    locationUpdate((Point) JSONUtils.getObjectWithKeyWord(message, Point.class));
                    break;
                case "arrivedWaypoint":
                    wayPointReached();
                    break;
                case "connect":
                    connectReceive();
                    break;
                default:
                    Log.logWarning("CORE", "No matching keyword when parsing JSON from Sockets. Data: " + message);
                    break;
            }
        }
    }

    //Event call over interface for when MQTT connection receives new route job requests. Adds all requested waypoints to route queue one by one.
    //Sets the vehicle to occupied. Ignores the request if vehicle is already occupied.
    private void jobRequest(int[] wayPointIDs) {
        Log.logInfo("CORE", "Route request received.");
        Boolean error = false;
        if (!occupied) {
            occupied = true;
            for (int wayPointID : wayPointIDs) {
                if (wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    Log.logInfo("CORE", "Added waypoint with ID " + wayPointID + " to route.");
                } else {
                    Log.logWarning("CORE", "Waypoint with ID '" + wayPointID + "' not found.");
                    currentRoute.clear();
                    routeError();
                    error = true;
                }

            }
            if(!error){
                routeSize = currentRoute.size();
                Log.logInfo("CORE", "All waypoints(" + routeSize + ") of route added. Starting route.");
                updateRoute();
            }
        } else {
            Log.logWarning("CORE", "Current Route not completed. Not adding waypoints.");
            routeNotComplete();
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {

        if (args.length == 0) {
            System.out.println("Need at least 1 argument to run. Possible arguments: startpoint(int)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) startPoint = Integer.parseInt(args[0]);
        }

        final Core core = new Core();
    }

}
package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;
import javax.ws.rs.client.Client;
import javax.ws.rs.client.ClientBuilder;
import javax.ws.rs.client.Invocation;
import javax.ws.rs.client.WebTarget;
import javax.ws.rs.core.Response;
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
import java.io.InputStream;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.Level;

//interface to trigger certain events from other objects such as TCP Sockets or MQTT.


public class Core implements CoreListener,MQTTListener {

    //Hardcoded elements.
    private boolean debugWithoutRos = false; // debug parameter to stop attempts to send over sockets when ROS-Node is active.
    private Log log;
    private Level level = Level.INFO; //Debug level
    Client client = ClientBuilder.newClient();
    WebTarget webTarget = client.target("http://localhost:8080/carmanager");
    private final String mqttBroker = "tcp://smartcity-ua.ddns.net:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final int serverPort = 5005;
    private final int clientPort = 5006;
    private final String mapFolder = "maps";

    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;

    private long ID; // ID given by RaceCarManager.
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private HashMap <Integer, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.
    private Queue<Integer> currentRoute = new LinkedList<>();// All waypoint IDs to be handled in the current route.
    private int routeSize = 0; // Current route's size.
    private boolean connected = false; // To verify socket connection to vehicle.
    private static int startPoint; // Starting position on map. Given by main argument.
    private boolean occupied = false; // To verify if racecar is currently occupied by a route job.

    public Core() throws InterruptedException, IOException {
        log = new Log(this.getClass(),level);
        register();
        mqttUtils = new MQTTUtils(ID,mqttBroker,mqqtUsername,mqttPassword,this);
        mqttUtils.subscribeToTopic("racecar/" + ID +"/#");
        tcpUtils = new TCPUtils(serverPort,clientPort,this);
        tcpUtils.start();
        if(!debugWithoutRos){
            connectSend();
            while(!connected){
                Thread.sleep(1);
            }
        }else{
            connected = true;
        }
        requestWaypoints();
        sendStartPoint();
        loadedMaps = XMLUtils.loadMaps(mapFolder);
        requestMap();
    }

    //Send connection check over sockets
    private void connectSend(){
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
    }

    //Event to be called when connection to car has been made
    public void connectReceive(){
        connected = true;
        Log.logInfo("CORE","Connected to car.");

    }

    //Register vehicle with RaceCarManager
    private void register(){
        WebTarget resourceWebTarget = webTarget.path("register");
        Invocation.Builder invocationBuilder = resourceWebTarget.request("text/plain");
        Response response = invocationBuilder.get();
        ID = Long.parseLong(response.readEntity(String.class), 10);
        Log.logInfo("CORE","Vehicle received ID " + ID + ".");
    }

    //Request all possible waypoints from RaceCarManager
    private void requestWaypoints(){
        WebTarget resourceWebTarget = webTarget.path("getwaypoints");
        Invocation.Builder invocationBuilder = resourceWebTarget.request("application/json");
        Response response = invocationBuilder.get();
        Type typeOfHashMap=new TypeToken<HashMap<Integer,WayPoint>>(){}.getType();
        wayPoints  = (HashMap<Integer, WayPoint>) JSONUtils.getObject(response.readEntity(String.class),typeOfHashMap);
        assert wayPoints != null;
        for (WayPoint wayPoint : wayPoints.values()) {
            Log.logConfig("CORE","Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
        }
        Log.logInfo("CORE","All possible waypoints(" + wayPoints.size() + ") received.");
    }

    //Set the starting point for the vehicle. Sending it over the socket connection.
    private void sendStartPoint(){
        Log.logInfo("CORE","Starting point set as waypoint with ID " + startPoint + ".");
        if(!debugWithoutRos)tcpUtils.sendUpdate(JSONUtils.objectToJSONString("startPoint",wayPoints.get(startPoint)));
    }



    //REST call to RaceCarManager to request the name of the current map. If this map is not found in the offline available maps, it does another
    //REST download call to download the map and store it in it's /mapFolder and add it to the maps.xml file.
    //After that it sends this information to the vehicle to be used over the socket connection.
    private void requestMap() throws IOException {
        WebTarget resourceWebTarget = webTarget.path("getmapname");
        Invocation.Builder invocationBuilder = resourceWebTarget.request("text/plain");
        Response response = invocationBuilder.get();
        String mapName = response.readEntity(String.class);
        //String mapName = "zbuilding";
        if(loadedMaps.containsKey(mapName)){
            Log.logInfo("CORE","Current map '" + mapName + "' found.");
            sendCurrentMap(mapName);
        }else{
            java.nio.file.Path out = Paths.get("maps/" + mapName + ".pgm");
            resourceWebTarget = webTarget.path("getmappgm/" + mapName);
            invocationBuilder = resourceWebTarget.request("application/octet-stream");
            response = invocationBuilder.get();
            InputStream in = response.readEntity(InputStream.class);
            Files.copy(in, out, StandardCopyOption.REPLACE_EXISTING);
            in.close();
            java.nio.file.Path out2 = Paths.get("maps/" + mapName + ".yaml");
            resourceWebTarget = webTarget.path("getmapyaml/" + mapName);
            invocationBuilder = resourceWebTarget.request("application/octet-stream");
            response = invocationBuilder.get();
            InputStream in2 = response.readEntity(InputStream.class);
            Files.copy(in2, out2, StandardCopyOption.REPLACE_EXISTING);
            in2.close();
            Log.logConfig("CORE","Current map '" + mapName + "' not found. Downloading...");
            addMap(mapName);
            Log.logInfo("CORE","Current map '" + mapName + "' downloaded.");
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
            Log.logSevere("CORE","Could not add map to XML of maps." + e);
        }
        loadedMaps.put(name,map);
        Log.logConfig("CORE","Added downloaded map : " + name + ".");

    }

    //Send the name and other information of the current map to the vehicle over the socket connection.
    private void sendCurrentMap(String mapName){
        if(!debugWithoutRos)tcpUtils.sendUpdate(JSONUtils.objectToJSONString("currentMap",loadedMaps.get(mapName)));

    }

    //Tto be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
    //Sends information of the next waypoint over the socket connection to the vehicle.
    private void updateRoute(){
        if(!currentRoute.isEmpty()){
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            if(!debugWithoutRos)tcpUtils.sendUpdate(JSONUtils.objectToJSONString("nextWayPoint",nextWayPoint));
            Log.logInfo("CORE","Sending next waypoint with ID " + nextWayPoint.getID() + " ("+ (routeSize - currentRoute.size()) + "/" + routeSize + ")");

        }else{
            routeCompleted();
        }
    }

    //Event call over interface to be used when socket connection received message that waypoint has been reached.
    public void wayPointReached(){
        Log.logInfo("CORE","Waypoint reached.");
        updateRoute();
    }

    //When all waypoints have been completed the vehicle becomes available again.
    private void routeCompleted(){
        Log.logInfo("CORE","Route Completed.");
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/routecomplete","done");
    }

    //Send wheel states to the vehicle over the socket connection. useful for emergency stops and other requests.
    private void sendWheelStates(float throttle, float steer) {
        if(!debugWithoutRos)tcpUtils.sendUpdate(JSONUtils.objectToJSONString("drive",new Drive(steer,throttle)));
        Log.logInfo("CORE","Sending wheel state Throttle:" + throttle +", Steer:" + steer + ".");
    }

    //Event call over interface for when the socket connection receives location update. Publishes this to the RaceCarManager over MQTT.
    public void locationUpdate(Point location) {
        Log.logInfo("CORE","Location Updated.");
        mqttUtils.publishMessage("racecar/" + ID + "/location", location.getX() + " " + location.getY() + " " + location.getZ() + " " + location.getW());
    }

    public void parseMQTT(String topic, String message){
        if(topic.equals("racecar/" + ID + "/job")){
            String[] waypointStringValues = message.split(" ");
            int[] waypointValues = new int[waypointStringValues.length];
            for (int index = 0; index < waypointStringValues.length; index++) {
                waypointValues[index] = Integer.parseInt(waypointStringValues[index]);
            }
            jobRequest(waypointValues);
        }
    }

    //Event call over interface for when MQTT connection receives new route job requests. Adds all requested waypoints to route queue one by one.
    //Sets the vehicle to occupied. Ignores the request if vehicle is already occupied.
    private void jobRequest(int[] wayPointIDs) {
       Log.logInfo("CORE","Route request received.");
        if(!occupied){
            occupied = true;
            for (int wayPointID : wayPointIDs) {
                if(wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    Log.logInfo("CORE","Added waypoint with ID " + wayPointID + " to route.");
                }else{
                    Log.logWarning("CORE","Waypoint with ID '" + wayPointID + "' not found.");
                }

            }
            routeSize = currentRoute.size();
            Log.logInfo("CORE","All waypoints(" + routeSize + ") of route added. Starting route.");
            updateRoute();

        }else{
            Log.logWarning("CORE","Current Route not completed. Not adding waypoints.");
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
package SmartRacecar;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
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
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;

//interface to trigger certain events from other objects such as TCP Sockets or MQTT.
interface CoreEvents {
    void locationUpdate(Point location);
    void jobRequest(int[] wayPoints);
    void wayPointReached();
    void connectReceive();
}

public class Core implements CoreEvents {

    //Hardcoded elements.
    private final static Logger logging = Logger.getLogger(Core.class.getName());
    private final static Level logLevel = Level.INFO;
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "username";
    private final String mqttPassword = "password";
    private final int serverPort = 5005;
    private final int clientPort = 5006;
    private final String mapFolder = "maps";

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;
    private JSONUtils jsonUtils;

    private int ID; // ID given by RaceCarManager.
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private HashMap <Integer, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.
    private Point currentLocation = new Point(0,0,0,0); // Most recent position of the vehicle.
    private Queue<Integer> currentRoute = new LinkedList<>();// All waypoint IDs to be handled in the current route.
    private int routeSize = 0; // Current route's size.
    private boolean connected = false; // To verify socket connection to vehicle.
    private static int startPoint; // Starting position on map. Given by main argument.
    private boolean occupied = false; // To verify if racecar is currently occupied by a route job.

    public Core() throws InterruptedException {
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(ID,mqttBroker,mqqtUsername,mqttPassword,this);
        tcpUtils = new TCPUtils(serverPort,clientPort,this);
        tcpUtils.start();
        restUtils = new RESTUtils("http://localhost:8080/x");
        connectSend();
        while(!connected){
            Thread.sleep(1);
        }
        register();
        requestWaypoints();
        sendStartPoint();
        loadMaps();
        requestMap();
    }

    //Send connection check over sockets
    private void connectSend(){
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
    }

    //Event to be called when connection to car has been made
    public void connectReceive(){
        connected = true;
        logInfo("CORE","Connected to car.");

    }

    //Register vehicle with RaceCarManager
    private void register(){
        //TODO add rest call to get ID
        ID = 0;
    }

    //Request all possible waypoints from RaceCarManager
    private void requestWaypoints(){
        //TODO request waypoints through REST
        WayPoint A = new WayPoint(1,(float)0.5,0,-1,(float)0.02,1);
        WayPoint B = new WayPoint(2,(float)-13.4,(float)-0.53,(float)0.71,(float)0.71,1);
        WayPoint C = new WayPoint(3,(float)-27.14,(float)-1.11,(float)-0.3,(float)0.95,1);
        WayPoint D = new WayPoint(4,(float)-28.25,(float)-9.19,(float)-0.71,(float)0.71,1);
        wayPoints.put(A.getID(),A);
        wayPoints.put(B.getID(),B);
        wayPoints.put(C.getID(),C);
        wayPoints.put(D.getID(),D);
        for (WayPoint wayPoint : wayPoints.values()) {
            logConfig("CORE","Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
        }

        logInfo("CORE","All possible waypoints(" + wayPoints.size() + ") received.");
    }

    //Set the starting point for the vehicle. Sending it over the socket connection.
    private void sendStartPoint(){
        tcpUtils.sendUpdate(JSONUtils.objectToJSONString("startPoint",wayPoints.get(startPoint)));
    }

    //Load all current available offline maps from the /mapFolder folder. It reads and maps.xml file with all the necessary information.
    private void loadMaps() {
        try {
            File fXmlFile = new File(mapFolder + "/maps.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fXmlFile);
            doc.getDocumentElement().normalize();

            NodeList nList = doc.getElementsByTagName("map");

            for (int temp = 0; temp < nList.getLength(); temp++) {

                Node nNode = nList.item(temp);

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element eElement = (Element) nNode;
                    String name = eElement.getElementsByTagName("name").item(0).getTextContent();
                    float meterPerPixel = Float.parseFloat(eElement.getElementsByTagName("meterPerPixel").item(0).getTextContent());
                    loadedMaps.put(name,new Map(name, meterPerPixel));
                    logConfig("CORE","Added map: " + name + ".");
                }
            }
        } catch (Exception e) {
            logSevere("CORE","Could not correctly load XML of maps." + e);
        }
    }

    //REST call to RaceCarManager to request the name of the current map. If this map is not found in the offline available maps, it does another
    //REST download call to download the map and store it in it's /mapFolder and add it to the maps.xml file.
    //After that it sends this information to the vehicle to be used over the socket connection.
    private void requestMap(){
        //TODO request map name through REST
        String mapName = "zbuilding";
        if(loadedMaps.containsKey(mapName)){
            logInfo("CORE","Current map '" + mapName + "' found.");
            sendCurrentMap(mapName);
        }else{
            //TODO download map from backbone
            logConfig("CORE","Current map '" + mapName + "' not found. Downloading...");
            addMap("test", (float) 0.05);
            logInfo("CORE","Current map '" + mapName + "' downloaded.");
            sendCurrentMap(mapName);
        }
    }

    //Add a new map to the /mapFolder folder and update the maps.xml file.
    private void addMap(String name, float meterPerPixel) {
        Map map = new Map(name, meterPerPixel);
        try {
            DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

            Document document = documentBuilder.parse(mapFolder + "/maps.xml");
            Element root = document.getDocumentElement();

            Element newMap = document.createElement("map");

            Element newName = document.createElement("name");
            newName.appendChild(document.createTextNode(name));
            newMap.appendChild(newName);

            Element newMeterPerPixel = document.createElement("meterPerPixel");
            newMeterPerPixel.appendChild(document.createTextNode(Float.toString(meterPerPixel)));
            newMap.appendChild(newMeterPerPixel);

            root.appendChild(newMap);

            DOMSource source = new DOMSource(document);

            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
            StreamResult result = new StreamResult(mapFolder + "/maps.xml");
            transformer.transform(source, result);

        } catch (ParserConfigurationException | SAXException | IOException | TransformerException e) {
            logSevere("CORE","Could not add map to XML of maps." + e);
        }
        loadedMaps.put(name,map);
        logConfig("CORE","Added downloaded map : " + name + ".");

    }

    //Send the name and other information of the current map to the vehicle over the socket connection.
    private void sendCurrentMap(String mapName){
        tcpUtils.sendUpdate(JSONUtils.objectToJSONString("currentMap",loadedMaps.get(mapName)));
    }

    //Tto be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
    //Sends information of the next waypoint over the socket connection to the vehicle.
    private void updateRoute(){
        if(!currentRoute.isEmpty()){
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            tcpUtils.sendUpdate(JSONUtils.objectToJSONString("nextWayPoint",nextWayPoint));
            logInfo("CORE","Sending next waypoint with ID " + nextWayPoint.getID() + " ("+ (routeSize - currentRoute.size()) + "/" + routeSize + ")");

        }else{
            routeCompleted();
        }
    }

    //Event call over interface to be used when socket connection received message that waypoint has been reached.
    public void wayPointReached(){
        logInfo("CORE","Waypoint reached.");
        updateRoute();
    }

    //When all waypoints have been completed the vehicle becomes available again.
    private void routeCompleted(){
        logInfo("CORE","Route Completed.");
        occupied = false;
        //TODO send message to backbone to complete route
    }

    //Send wheel states to the vehicle over the socket connection. useful for emergency stops and other requests.
    private void sendWheelStates(float throttle, float steer) {
        tcpUtils.sendUpdate(JSONUtils.objectToJSONString("drive",new Drive(steer,throttle)));
        logInfo("CORE","Sending wheel state Throttle:" + throttle +", Steer:" + steer + ".");
    }

    //Event call over interface for when the socket connection receives location update. Publishes this to the RaceCarManager over MQTT.
    public void locationUpdate(Point location) {
        currentLocation = location;
        logInfo("CORE","Location Updated.");
        mqttUtils.publishMessage("racecar/" + ID + "/location", currentLocation.getX() + " " + currentLocation.getY() + " " + currentLocation.getZ() + " " + currentLocation.getW());
    }

    //Event call over interface for when MQTT connection receives new route job requests. Adds all requested waypoints to route queue one by one.
    //Sets the vehicle to occupied. Ignores the request if vehicle is already occupied.
    public void jobRequest(int[] wayPointIDs) {
        logInfo("CORE","Route request received.");
        if(!occupied){
            occupied = true;
            for (int wayPointID : wayPointIDs) {
                if(wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    logInfo("CORE","Added waypoint with ID " + wayPointID + " to route.");
                }else{
                    logWarning("CORE","Waypoint with ID '" + wayPointID + "' not found.");
                }

            }
            routeSize = currentRoute.size();
            logInfo("CORE","All waypoints(" + routeSize + ") of route added. Starting route.");
            updateRoute();

        }else{
            logWarning("CORE","Current Route not completed. Not adding waypoints.");
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        setLogger(logLevel);

        if (args.length == 0) {
            logSevere("CORE", "Need at least 1 argument to run. Possible arguments: startpoint(int)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) startPoint = Integer.parseInt(args[0]);
            logInfo("CORE", "Starting point set as waypoint with ID " + startPoint + ".");
        }

        final Core core = new Core();
//    }
    }

    //Logging functionality for debugging and information printing. 
    static void logInfo(String category, String message){
        logging.info("[" + category + "] " + message);
    }
    static void logWarning(String category, String message){
        logging.warning("[" + category + "] " + message);
    }
    static void logSevere(String category, String message){
        logging.severe("[" + category + "] " + message);
    }
    static void logConfig(String category, String message){
        logging.config("[" + category + "] " + message);
    }
    private static void setLogger(Level level){
        Handler handlerObj = new ConsoleHandler();
        handlerObj.setLevel(Level.ALL);
        LogFormatter logFormatter = new LogFormatter();
        handlerObj.setFormatter(logFormatter);
        logging.addHandler(handlerObj);
        logging.setLevel(Level.ALL);
        logging.setUseParentHandlers(false);
        logging.setLevel(level);
    }
}
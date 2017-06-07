package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Map;
import com.google.gson.reflect.TypeToken;
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
import java.io.*;
import java.lang.reflect.Type;
import java.util.*;
import java.util.logging.Level;

/**
 * Module representing the high-level of a vehicle.
 */
class Core implements TCPListener, MQTTListener {
    //Hardcoded settings
    private boolean debugWithoutRosKernel = false; // debug parameter for using this module without a connected RosKernel/SimKernel
    private Level level = Level.INFO; //Logging level. Best usages are LEVEL.INFO(for basic info) and LEVEL.CONFIG(for debug messages).
    private final String mqttBroker = "tcp://143.129.39.151:1883"; // MQTT Broker URL
    private final String mqqtUsername = "root"; // MQTT Broker Username
    private final String mqttPassword = "smartcity"; // MQTT Broker Password
    private final String restURL = "http://143.129.39.151:8081/carmanager"; // REST Service URL to Manager
    private static int serverPort = 5005; // Standard TCP Port to listen on for messages from ROS Node.
    private static int clientPort = 5006; // Standard TCP Port to send to messages to ROS Node.

    //Help services
    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    //variables
    private long ID; // ID given by Manager to identify vehicle.
    private int routeSize = 0; // Current route's size.
    private static long startPoint; // Starting position on map. Given by main argument.
    private HashMap<String, Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.
    private Queue<Long> currentRoute = new LinkedList<>(); // All waypoint IDs to be handled in the current route.
    private int CostCurrentToStartTiming = -1; // Time in seconds from current position to start position of route.
    private int CostStartToEndTiming = -1; // Time in seconds from start position to end position of route.
    private boolean connected = false; // To verify socket connection to vehicle.
    private boolean occupied = false; // To verify if racecar is currently occupied by a route job.


    /**
     * Module representing the high-level of a vehicle.
     *
     * @param startPoint Starting point of the vehicle. Defined by input arguments of main method.
     * @param serverPort Port to listen for messages of SimKernel/Roskernel. Defined by input arguments of main method.
     * @param clientPort Port to send messages to SimKernel/Roskernel. Defined by input arguments of main method.
     */
    private Core(long startPoint, int serverPort, int clientPort) throws InterruptedException, IOException {
        Log log = new Log(this.getClass(), level);
        Log.logConfig("CORE", "Startup parameters: Starting Waypoint:" + startPoint + " | TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
        restUtils = new RESTUtils(restURL);
        requestWaypoints();
        register();
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/" + ID + "/#");
        tcpUtils = new TCPUtils(clientPort, serverPort, this, false);
        tcpUtils.start();
        if (!debugWithoutRosKernel) {
            connectSend();
        }
        sendStartPoint();
        loadedMaps = loadMaps(findMapsFolder());
        requestMap();
        /*Properties prop = new Properties();
        InputStream input = null;

        try {

            input = new FileInputStream("config.properties");

            // load a properties file
            prop.load(input);

            // get the property value and print it out
            System.out.println(prop.getProperty("database"));
            System.out.println(prop.getProperty("dbuser"));
            System.out.println(prop.getProperty("dbpassword"));

        } catch (IOException ex) {
            Log.logConfig("CORE", "Could not read config file: " + ex);

            ex.printStackTrace();
        } finally {
            if (input != null) {
                try {
                    input.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }*/
    }

    /**
     * Load all current available offline maps from the /mapFolder folder.
     * It reads the maps.xml file with all the necessary information.
     *
     * @param mapFolder location of the maps.xml file.
     * @return Returns a Hashmap<String,Map> where the String is the mapname. It contains all loaded maps.
     */
    private HashMap<String, Map> loadMaps(String mapFolder) {
        HashMap<String, Map> loadedMaps = new HashMap<>();
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
                    loadedMaps.put(name, new Map(name));
                    Log.logConfig("XML", "Added map: " + name + ".");
                }
            }
        } catch (Exception e) {
            Log.logSevere("XML", "Could not correctly load XML of maps." + e);
        }
        return loadedMaps;
    }

    /**
     * Find the location of the maps.xml containing folder. Searches up to 3 levels deep.
     *
     * @return  returns a String containing the location of the maps.xml's absolute path.
     */
    private String findMapsFolder() {
        FileUtils fileUtils = new FileUtils();
        fileUtils.searchDirectory(new File(".."), "maps.xml");
        if (fileUtils.getResult().size() == 0) {
            fileUtils.searchDirectory(new File("./.."), "maps.xml");
            if (fileUtils.getResult().size() == 0) {
                fileUtils.searchDirectory(new File("./../.."), "maps.xml");
                if (fileUtils.getResult().size() == 0) {
                    fileUtils.searchDirectory(new File("./../../.."), "maps.xml");
                    if (fileUtils.getResult().size() == 0) {
                        Log.logSevere("CORE", "maps.xml not found. Make sure it exists in some folder (maximum 3 levels deep).");
                        System.exit(0);
                    }
                }
            }
        }
        String output = null;
        for (String matched : fileUtils.getResult()) {
            output = matched;
        }
        return output;
    }

    /**
     * Send connection request over sockets to RosKernel/SimKernel.
     */
    private void connectSend() {
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
    }

    /**
     * Event to be called when connection to car has been made.
     */
    private void connectReceive() {
        Log.logInfo("CORE", "Connected to car.");
    }


    /**
     * Register vehicle with Manager over REST.
     */
    private void register() {
        String id = restUtils.getTextPlain("register/" + Long.toString(startPoint));
        ID = Long.parseLong(id, 10);
        Log.logInfo("CORE", "Vehicle received ID " + ID + ".");
    }

    /**
     * Request all possible waypoints from RaceCarManager over REST
     */
    private void requestWaypoints() {
        Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>() {
        }.getType();
        wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
        assert wayPoints != null;
        for (WayPoint wayPoint : wayPoints.values()) {
            Log.logConfig("CORE", "Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
        }
        Log.logInfo("CORE", "All possible waypoints(" + wayPoints.size() + ") received.");
    }

    /**
     * Sends starting point to the vehicle's SimKernel/RosKernel over socket connection.
     */
    private void sendStartPoint() {
        Log.logInfo("CORE", "Starting point set as waypoint with ID " + startPoint + ".");
        if (!debugWithoutRosKernel)
            tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startPoint", wayPoints.get(startPoint)));
    }

    /**
     * REST GET request to Manager to request the name of the current map. If this map is not found in the offline available maps, it does another
     * REST GET request to download the map files and store it in the mapfolder and add it to the maps.xml file.
     * After that it sends this information to the vehicle SimKernel/SimKernel over the socket connection.
     */
    private void requestMap() throws IOException {
        String mapName = restUtils.getTextPlain("getmapname");
        //String mapName = "zbuilding";
        if (loadedMaps.containsKey(mapName)) {
            Log.logInfo("CORE", "Current used map '" + mapName + "' found in folder, setting as current map.");
            if (!debugWithoutRosKernel)
                tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentMap", loadedMaps.get(mapName)));
        } else {
            Log.logConfig("CORE", "Current used map '" + mapName + "' not found. Downloading...");
            restUtils.getFile("getmappgm/" + findMapsFolder(), "maps", mapName, "pgm");
            restUtils.getFile("getmapyaml/" + findMapsFolder(), "maps", mapName, "yaml");
            Map map = new Map(mapName);
            try {
                DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
                DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

                Document document = documentBuilder.parse(findMapsFolder() + "/maps.xml");
                Element root = document.getDocumentElement();

                Element newMap = document.createElement("map");

                Element newName = document.createElement("name");
                newName.appendChild(document.createTextNode(mapName));
                newMap.appendChild(newName);

                root.appendChild(newMap);

                DOMSource source = new DOMSource(document);

                TransformerFactory transformerFactory = TransformerFactory.newInstance();
                Transformer transformer = transformerFactory.newTransformer();
                transformer.setOutputProperty(OutputKeys.INDENT, "yes");
                transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
                StreamResult result = new StreamResult(findMapsFolder() + "/maps.xml");
                transformer.transform(source, result);

            } catch (ParserConfigurationException | SAXException | IOException | TransformerException e) {
                Log.logWarning("CORE", "Could not add map to XML of maps." + e);
            }
            loadedMaps.put(mapName, map);
            Log.logConfig("CORE", "Added downloaded map : " + mapName + ".");
            Log.logInfo("CORE", "Current map '" + mapName + "' downloaded and set as current map.");
            if (!debugWithoutRosKernel)
                tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentMap", loadedMaps.get(mapName)));
        }
    }

    /**
     * To be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
     * Sends information of the next waypoint over the socket connection to the vehicle's SimKernel/RosKernel.
     */
    private void updateRoute() {
        if (!currentRoute.isEmpty()) {
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            if (!debugWithoutRosKernel)
                tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("nextWayPoint", nextWayPoint));
            Log.logInfo("CORE", "Sending next waypoint with ID " + nextWayPoint.getID() + " (" + (routeSize - currentRoute.size()) + "/" + routeSize + ")");
            if (debugWithoutRosKernel) { //Debug code to go over all waypoints with a 3s sleep in between.
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

    /**
     * Event call over interface to be used when socket connection received message that waypoint has been reached.
     */
    private void wayPointReached() {
        Log.logInfo("CORE", "Waypoint reached.");
        updateRoute();
    }

    /**
     * When all waypoints have been completed the vehicle becomes unoccupied again.
     * Sends MQTT message to Manager to update the vehicle status.
     */
    private void routeCompleted() {
        Log.logInfo("CORE", "Route Completed.");
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "done");
    }

    /**
     * When all a requested route job can't be done by the vehicle.
     * Sends MQTT message to Manager to update the route status.
     */
    private void routeError() {
        Log.logWarning("CORE", "Route error. Route Cancelled");
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "error");
    }

    /**
     * When all a requested route job can't be done by the vehicle as it's still completing a route.
     * Sends MQTT message to Manager to update the route status.
     */
    private void routeNotComplete() {
        occupied = false;
        mqttUtils.publishMessage("racecar/" + ID + "/route", "notcomplete");
    }

    /**
     * Send wheel states to the vehicle over the socket connection. useful for emergency stops and other specific requests.
     *
     * @param throttle throttle value for the vehicle wheels.
     * @param steer    rotation value for the vehicle wheels.
     */
    private void sendWheelStates(float throttle, float steer) {
        if (!debugWithoutRosKernel)
            tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("drive", new Drive(steer, throttle)));
        Log.logInfo("CORE", "Sending wheel state Throttle:" + throttle + ", Steer:" + steer + ".");
    }


    /**
     * When vehicle's RosKernel/SimKernel sends update on route completion it needs to be transformed to the completion amount for the total route.
     * To do this it receives a cost calculation from the RosServer that it requested and uses this to determine the size of each waypoint's sub-route.
     *
     * @param location Location object containing the current percentage value.
     */
    private void locationUpdate(Location location) {
        if (currentRoute.size() == 0) {
            float weight = (float) CostStartToEndTiming / (float) (CostCurrentToStartTiming + CostStartToEndTiming);
            location.setPercentage(Math.round((1 - weight) * 100 + location.getPercentage() * weight));
        } else if (currentRoute.size() == 1) {

            float weight = (float) CostCurrentToStartTiming / (float) (CostCurrentToStartTiming + CostStartToEndTiming);
            location.setPercentage(Math.round(location.getPercentage() * weight));
        }
        Log.logInfo("CORE", "Location Updated. Vehicle has " + location.getPercentage() + "% of route completed");
        mqttUtils.publishMessage("racecar/" + ID + "/percentage", JSONUtils.objectToJSONString(location));
    }

    /**
     * Interfaced method to parse MQTT message and topic after MQTT callback is triggered by incoming message.
     * Used by messages coming from Manager to request a route job or a cost calculation request.
     *
     * @param topic   received MQTT topic
     * @param message received MQTT message string
     */
    public void parseMQTT(String topic, String message) {
        if (topic.matches("racecar/[0-9]+/job")) {
            if (message.equals("stop")) {
                sendWheelStates(0, 0);
            } else {
                if (!occupied) {
                    String[] wayPointStringValues = message.split(" ");
                    try {
                        long[] wayPointValues = new long[wayPointStringValues.length];
                        for (int index = 0; index < wayPointStringValues.length; index++) {

                            wayPointValues[index] = Integer.parseInt(wayPointStringValues[index]);
                        }
                        jobRequest(wayPointValues);
                    } catch (NumberFormatException e) {
                        Log.logWarning("CORE", "Parsing MQTT gives bad result: " + e);
                    }
                } else {
                    Log.logWarning("CORE", "Current Route not completed. Not adding waypoints.");
                    routeNotComplete();
                }
            }
        } else if (topic.matches("racecar/[0-9]+/costrequest")) {
            String[] wayPointStringValues = message.split(" ");
            try {
                long[] wayPointValues = new long[wayPointStringValues.length];
                for (int index = 0; index < wayPointStringValues.length; index++) {

                    wayPointValues[index] = Integer.parseInt(wayPointStringValues[index]);
                }
                costRequest(wayPointValues);
            } catch (NumberFormatException e) {
                Log.logWarning("CORE", "Parsing MQTT gives bad result: " + e);
            }
        }
    }

    /**
     * Interfaced method to parse TCP message socket callback is triggered by incoming message.
     * Used for messages about percentage route completion, next waypoint arrived, initial connection setup,
     * stopping/killing/starting/restarting vehicle, setting the startpoint or cost calculation response.
     *
     * @param message received TCP socket message string
     * @return a return answer to be send back over the socket to the SimKernel/RosKernel
     */
    public String parseTCP(String message) {
        if (JSONUtils.isJSONValid(message)) {
            //parses keyword to do the correct function call.
            switch (JSONUtils.getFirst(message)) {
                case "percentage":
                    locationUpdate((Location) JSONUtils.getObjectWithKeyWord(message, Location.class));
                    break;
                case "arrivedWaypoint":
                    wayPointReached();
                    break;
                case "connect":
                    connectReceive();
                    break;
                case "kill":
                    killCar();
                    break;
                case "stop":
                    sendAvailability(false);
                    break;
                case "start":
                    sendAvailability(true);
                    break;
                case "startpoint":
                    startPoint = (long) JSONUtils.getObjectWithKeyWord(message, Long.class);
                    mqttUtils.publishMessage("racecar/" + ID + "/locationupdate", Long.toString((Long) JSONUtils.getObjectWithKeyWord(message, Long.class)));
                    Log.logInfo("CORE", "Setting new starting point with ID " + JSONUtils.getObjectWithKeyWord(message, Long.class));
                    break;
                case "restart":
                    sendAvailability(true);
                    tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentPosition", wayPoints.get(startPoint)));
                    Log.logInfo("CORE", "Vehicle restarted.");
                    break;
                case "cost":
                    costComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
                    break;
                case "costtiming":
                    timeComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
                    break;
                default:
                    Log.logWarning("CORE", "No matching keyword when parsing JSON from Sockets. Data: " + message);
                    break;
            }
        }
        return null;
    }

    /**
     * Set availability status of vehicle in the Manager by sending MQTT message.
     *
     * @param state state to be send. (available=true, unavailable=false)
     */
    private void sendAvailability(boolean state) {
        mqttUtils.publishMessage("racecar/" + ID + "/available", Boolean.toString(state));
        Log.logInfo("CORE", "Vehicle's availability status set to " + state + '.');
    }

    /**
     * Closes all connections (TCP & MQTT), unregisters the vehicle with the Manager and shut the module down.
     */
    private void killCar() {
        Log.logInfo("CORE", "Vehicle kill request. Closing connections and shutting down...");
        restUtils.getCall("delete/" + ID);
        if (!debugWithoutRosKernel) tcpUtils.closeTCP();
        mqttUtils.closeMQTT();
        System.exit(0);
    }

    /**
     * Called by incoming cost calculation requests. Sends the request further to the RosKernel/SimKernel.
     *
     * @param wayPointIDs Array of waypoint ID's to have their cost calculated.
     */
    private void costRequest(long[] wayPointIDs) {
        List<Point> points = new ArrayList<>();
        points.add(wayPoints.get(wayPointIDs[0]));
        points.add(wayPoints.get(wayPointIDs[1]));
        if (!debugWithoutRosKernel) {
            tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("cost", points));
        } else {
            costComplete(new Cost(false, 5, 5, ID));
        }
        Log.logInfo("CORE", "Cost request received between waypoints " + wayPointIDs[0] + " and " + wayPointIDs[1] + ". Calculating.");
    }

    /**
     * Called by received response from SimKernel/RosKernel of cost calculation request.
     *
     * @param cost Cost object containing the calculated weights.
     */
    private void costComplete(Cost cost) {
        Log.logInfo("CORE", "Cost request calculated.");
        cost.setStatus(occupied);
        cost.setIdVehicle(ID);
        mqttUtils.publishMessage("racecar/" + ID + "/costanswer", JSONUtils.objectToJSONString(cost));
    }

    /**
     * Called by incoming timing calculation requests. Sends the request further to the RosKernel/SimKernel.
     *
     * @param wayPointIDs Array of waypoint ID's to have their timing calculated.
     */
    private void timeRequest(long[] wayPointIDs) {
        List<Point> points = new ArrayList<>();
        points.add(wayPoints.get(wayPointIDs[0]));
        points.add(wayPoints.get(wayPointIDs[1]));
        if (!debugWithoutRosKernel) {
            tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("costtiming", points));
        } else {
            costComplete(new Cost(false, 5, 5, ID));
        }
    }

    /**
     * When vehicle has completed cost calculation for the locationUpdate() function it's sets the variables
     * CostCurrentToStartTiming and CostStartToEndTiming of the Core to be used by locationUpdate().
     *
     * @param cost Cost object containing the weights of the sub-routes.
     */
    private void timeComplete(Cost cost) {
        CostCurrentToStartTiming = cost.getWeightToStart();
        CostStartToEndTiming = cost.getWeight();
    }

    /**
     * Event call over interface for when MQTT connection receives new route job requests from the Manager.
     * Adds all requested waypoints to route queue one by one.
     * Sets the vehicle to occupied. Ignores the request if vehicle is already occupied.
     * Then triggers the first waypoint to be send to the RosKernel/SimKernel.
     *
     * @param wayPointIDs Array of waypoint ID's that are on the route to be completed.
     */
    private void jobRequest(long[] wayPointIDs) {
        Log.logInfo("CORE", "Route request received.");
        Boolean error = false;
        if (!occupied) {
            occupied = true;
            timeRequest(wayPointIDs);
            while (CostCurrentToStartTiming == -1 && CostStartToEndTiming == -1) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            for (long wayPointID : wayPointIDs) {
                if (wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    Log.logInfo("CORE", "Added waypoint with ID " + wayPointID + " to route.");
                } else {
                    Log.logWarning("CORE", "Waypoint with ID '" + wayPointID + "' not found.");
                    currentRoute.clear();
                    error = true;
                }
            }
            if (!error) {
                routeSize = currentRoute.size();
                Log.logInfo("CORE", "All waypoints(" + routeSize + ") of route added. Starting route.");
                updateRoute();
            } else {
                Log.logWarning("CORE", "Certain waypoints not found. Route cancelled.");
                routeError();
            }
        } else {
            Log.logWarning("CORE", "Current Route not completed. Not adding waypoints.");
            routeNotComplete();
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {

        if (args.length == 0) {
            System.out.println("Need 1 or 3 argument to run. Possible arguments: startpoint(int)(needed!) tcpclientport(int) tcpserverport(int)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) startPoint = Long.parseLong(args[0]);
        } else if (args.length == 2) {
            System.out.println("Need at least 1 or 3 argument to run. Possible arguments: startpoint(int)(needed!) tcpclientport(int) tcpserverport(int)");
            System.exit(0);
        } else {
            if (!args[0].isEmpty()) startPoint = Long.parseLong(args[0]);
            if (!args[1].isEmpty()) serverPort = Integer.parseInt(args[1]);
            if (!args[2].isEmpty()) clientPort = Integer.parseInt(args[2]);
        }
        final Core core = new Core(startPoint, serverPort, clientPort);
    }

}
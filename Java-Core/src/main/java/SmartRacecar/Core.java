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

interface CoreEvents {
    void locationUpdate(float x,float y);
    void jobRequest(int[] wayPoints);
    void updateRoute();
    void connectReceive();
}

public class Core implements CoreEvents {

    private static Logger logging = Logger.getLogger(Core.class.getName());

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;
    private JSONUtils jsonUtils;
    private int ID; // ID given by SmartCity Core
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // list of all loaded maps
    private HashMap <Integer, WayPoint> wayPoints = new HashMap<>();
    private Point currentLocation = new Point(0,0,0,0); // most recent position of the vehicle
    private Queue<Integer> currentRoute = new LinkedList<>();// all waypoints to be handled in the current route
    private String mapFolder = "maps";
    private int passengers = 0; // amount of passengers inside //TODO implement systems for passengers
    private boolean connected = false;
    private static int startPoint = 0;

    public Core() throws InterruptedException {
        setLogger(Level.INFO);
        logInfo("CORE","Starting point: " + startPoint);
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(ID,"tcp://broker.hivemq.com:1883","username","password",this);
        tcpUtils = new TCPUtils(5005,5006,this);
        tcpUtils.start();
        restUtils = new RESTUtils("http://localhost:8080/x");
        connectSend();

        while(!connected){
            Thread.sleep(1);
        }
        register();
        requestWaypoints();
        loadMapsFromFolder();
        sendStartPoint();
        requestMap();



        while (true) {
              Thread.sleep(1000);
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        startPoint = Integer.parseInt(args[0]);
        new Core();
    }

    private void setLogger(Level level){
        Handler handlerObj = new ConsoleHandler();
        handlerObj.setLevel(Level.ALL);
        LogFormatter logFormatter = new LogFormatter();
        handlerObj.setFormatter(logFormatter);
        logging.addHandler(handlerObj);
        logging.setLevel(Level.ALL);
        logging.setUseParentHandlers(false);
        logging.setLevel(level);
    }

    private void connectSend(){
        tcpUtils.sendUpdate(jsonUtils.keywordToJSONString("connect"));
    }

    public void connectReceive(){
        connected = true;
        logInfo("CORE","Connected to car.");

    }

    public void updateRoute(){
        if(!currentRoute.isEmpty()){
            WayPoint nextWayPoint = wayPoints.get(currentRoute.poll());
            tcpUtils.sendUpdate(jsonUtils.objectToJSONString("nextWayPoint",nextWayPoint));
            logInfo("CORE","Waypoint reached. Sending next one: " + nextWayPoint.getX() + "," + nextWayPoint.getY() + "," + nextWayPoint.getZ() + "," + nextWayPoint.getW());
        }else{
            routeCompleted();
        }
    }

    public static void logInfo(String category,String message){
        logging.info("[" + category + "] " + message);
    }

    public static void logWarning(String category, String message){
        logging.warning("[" + category + "] " + message);
    }

    public static void logSevere(String category, String message){
        logging.severe("[" + category + "] " + message);
    }

    public static void logConfig(String category, String message){
        logging.config("[" + category + "] " + message);
    }

    private void routeCompleted(){
        logInfo("CORE","Waypoint reached. Route Completed.");
        //TODO send message to backbone to complete route
    }

    private void register(){
        //TODO add rest call to get ID
        ID = 0;
    }

    private void sendWheelStates(float throttle, float steer) {
        tcpUtils.sendUpdate(jsonUtils.objectToJSONString("drive",new Drive(steer,throttle)));
    }

    private void exitCore(){
        sendWheelStates(0,0);
        tcpUtils.run = false;
        tcpUtils.closeTCP();
        mqttUtils.closeMQTT();
        System.exit(0);
    }

    public void locationUpdate(float x,float y) {
        logInfo("CORE","Location Updated.");
        currentLocation.setPoint(x,y);
        mqttUtils.publishMessage(ID + "/location", x + "," + y);
    }

    private void requestMap(){
        //TODO request map name through REST
        String mapName = "zbuilding";
        if(loadedMaps.containsKey(mapName)){
            logInfo("CORE","Map '" + mapName + "' found. Setting as current map.");
            sendCurrentMap(mapName);
        }else{
            //TODO download map from backbone
            logConfig("CORE","Map '" + mapName + "' not found. Downloading...");
            addMap("test", (float) 0.05);
            logInfo("CORE","Map '" + mapName + "' downloaded. Setting as current map.");
            sendCurrentMap(mapName);
            sendStartPoint();
        }
    }

    private void sendCurrentMap(String mapName){
        tcpUtils.sendUpdate(jsonUtils.objectToJSONString("currentMap",loadedMaps.get(mapName)));
    }

    private void sendStartPoint(){
        tcpUtils.sendUpdate(jsonUtils.objectToJSONString("startPoint",wayPoints.get(startPoint)));
    }

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

        logInfo("CORE","All waypoints received.");
    }

    @Override
    public void jobRequest(int[] wayPointIDs) {
        if(currentRoute.isEmpty()){
            for (int wayPointID : wayPointIDs) {
                if(wayPoints.containsKey(wayPointID)) {
                    currentRoute.add(wayPointID);
                    logInfo("CORE","Added waypoint " + wayPointID + " at " + wayPoints.get(wayPointID).getX() + "," + wayPoints.get(wayPointID).getY() + "," + wayPoints.get(wayPointID).getZ() + "," + wayPoints.get(wayPointID).getW() + " to route.");
                }else{
                    logWarning("CORE","[Core] [ERROR] Waypoint with ID '" + wayPointID + "' not found.");
                }

            }
            logInfo("CORE","sending car first waypoint: " + wayPoints.get(currentRoute.peek()).getX() + "," + wayPoints.get(currentRoute.peek()).getX() + "," + wayPoints.get(currentRoute.peek()).getZ() + "," + wayPoints.get(currentRoute.peek()).getW() + " to route.");

            updateRoute();

        }else{
            logWarning("CORE","Current Route not completed. Not adding waypoints.");
        }
    }

    private void loadMapsFromFolder() {

        try {

            File fXmlFile = new File(mapFolder + "/maps.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(fXmlFile);

            //optional, but recommended
            //read this - http://stackoverflow.com/questions/13786607/normalization-in-dom-parsing-with-java-how-does-it-work
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
            e.printStackTrace();
        }
    }

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
            e.printStackTrace();
        }
        loadedMaps.put(name,map);
        logConfig("CORE","Added downloaded map : " + name + ".");

    }
}
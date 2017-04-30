package SmartRacecar;

import org.json.simple.JSONObject;
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

interface eventListener {
    void locationUpdate(float x,float y);
    void jobRequest(int[] wayPoints);
    void updateRoute();
}

public class Core implements eventListener {

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private TCPUtils tcpUtils;
    private int ID; // ID given by SmartCity Core
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // list of all loaded maps
    private HashMap <Integer, WayPoint> wayPoints = new HashMap<>();
    private Point currentLocation = new Point(0,0); // most recent position of the vehicle
    private Queue<Integer> currentRoute = new LinkedList<>();// all waypoints to be handled in the current route
    private String mapFolder = "maps";
    private int passengers = 0; // amount of passengers inside //TODO implement systems for passengers

    public Core() throws InterruptedException {
        mqttUtils = new MQTTUtils(ID,"tcp://broker.hivemq.com:1883","username","password",this);
        tcpUtils = new TCPUtils(5005,5006,this);
        tcpUtils.start();
        restUtils = new RESTUtils("http://localhost:8080/x");

        register();
        loadMapsFromFolder();
        requestMap();
        requestWaypoints();


        while (true) {
              Thread.sleep(1000);
//            sendWheelStates(0,20);
//            Thread.sleep(1000);
//            sendWheelStates(0,-20);
//            Thread.sleep(1000);
//            sendWheelStates(1,0);
//            Thread.sleep(1000);
//            sendWheelStates(2,0);
//            Thread.sleep(1000);
//            sendWheelStates(0,0);
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

    private void routeCompleted(){
        System.out.println("[Core] [DEBUG] Route completed");
        //TODO send message to backbone to complete route
    }

    public static void main(String[] args) throws IOException, InterruptedException {
       new Core();
    }

    private void register(){
        //TODO add rest call to get ID
        ID = 0;
    }

    private void sendWheelStates(float throttle, float steer) {
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("throttle", throttle);
        childData.put("steer", steer);
        parentData.put("drive", childData);
        tcpUtils.sendUpdate(JSONUtils.JSONtoString(parentData));
    }

    private void exitCore(){
        sendWheelStates(0,0);
        tcpUtils.run = false;
        tcpUtils.closeTCP();
        mqttUtils.closeMQTT();
        System.exit(0);
    }

    public void locationUpdate(float x,float y) {
        System.out.println("[CORE] [DEBUG] LOCATION UPDATED");
        currentLocation.setPoint(x,y);
        mqttUtils.publishMessage(ID + "/location", x + "," + y);
    }

    private void requestMap(){
        //TODO request map name through REST
        String mapName = "V314";
        if(loadedMaps.containsKey(mapName)){
            System.out.println("[Core] [DEBUG] Map '" + mapName + "' found. Setting as current map.");
            sendCurrentMap(mapName);
        }else{
            //TODO download map from backbone
            System.out.println("[Core] [DEBUG] Map '" + mapName + "' not found. Downloading...");
            addMap("test",0,0, (float) 0.25);
            System.out.println("[Core] [DEBUG] Map '" + mapName + "' downloaded. Setting as current map.");
            sendCurrentMap(mapName);
        }
    }

    private void sendCurrentMap(String mapName){
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("name", mapName);
        childData.put("startPointX", loadedMaps.get(mapName).getStartPoint().getX());
        childData.put("startPointY", loadedMaps.get(mapName).getStartPoint().getY());
        childData.put("meterPerPixel", loadedMaps.get(mapName).getMeterPerPixel());
        parentData.put("currentMap", childData);
        tcpUtils.sendUpdate(JSONUtils.JSONtoString(parentData));
    }

    private void requestWaypoints(){
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
                    float x = Float.parseFloat(eElement.getElementsByTagName("startPointX").item(0).getTextContent());
                    float y = Float.parseFloat(eElement.getElementsByTagName("startPointY").item(0).getTextContent());
                    float meterPerPixel = Float.parseFloat(eElement.getElementsByTagName("meterPerPixel").item(0).getTextContent());
                    loadedMaps.put(name,new Map(name, new Point(x, y), meterPerPixel));
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void addMap(String name, float x, float y, float meterPerPixel) {
        Map map = new Map(name, new Point(x, y), meterPerPixel);
        try {
            DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

            Document document = documentBuilder.parse(mapFolder + "/maps.xml");
            Element root = document.getDocumentElement();

            Element newMap = document.createElement("map");

            Element newName = document.createElement("name");
            newName.appendChild(document.createTextNode(name));
            newMap.appendChild(newName);

            Element newStartPointX = document.createElement("startPointX");
            newStartPointX.appendChild(document.createTextNode(Float.toString(x)));
            newMap.appendChild(newStartPointX);

            Element newStartPointY = document.createElement("startPointY");
            newStartPointY.appendChild(document.createTextNode(Float.toString(y)));
            newMap.appendChild(newStartPointY);

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

    }
}
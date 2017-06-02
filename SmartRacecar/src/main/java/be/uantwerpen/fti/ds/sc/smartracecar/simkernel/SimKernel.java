package be.uantwerpen.fti.ds.sc.smartracecar.simkernel;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;

class SimKernel implements TCPListener {

    private boolean debugWithoutRosServer = false; // debug parameter to stop attempts to send over sockets when ROSServer-Node is active.
    private Log log;
    private Level level = Level.CONFIG; //Debug level
    private final String restURL = "http://143.129.39.117:8080";
    private static int serverPort = 5005;
    private static int clientPort = 5006;

    private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    private boolean connected = false; // To verify socket connection to vehicle.
    private Map map;
    private WayPoint startPoint;
    private Point currentPosition;


    SimKernel(int serverPort,int clientPort) throws InterruptedException {
        log = new Log(this.getClass(), level);
        log.logConfig("SIMKERNEL","Startup parameters: TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
        restUtils = new RESTUtils(restURL);
        tcpUtils = new TCPUtils(clientPort, serverPort, this,false);
        tcpUtils.start();
        while (!connected) {
            log.logWarning("SIMKERNEL","Waiting for connection with vehicle Core...");
            Thread.sleep(1000);
        }
    }

    @Override
    public String parseTCP(String message) {
        if (JSONUtils.isJSONValid(message)) {
            //parses keyword to do the correct function call.
            switch (JSONUtils.getFirst(message)) {
                case "cost":
                    Type typeOfPoints = new TypeToken<ArrayList<Point>>() {}.getType();
                    calculateCost((ArrayList<Point>) JSONUtils.getObjectWithKeyWord(message,typeOfPoints));
                    break;
                case "costtiming":
                    Type typeOfPointss = new TypeToken<ArrayList<Point>>() {}.getType();
                    calculateTiming((ArrayList<Point>) JSONUtils.getObjectWithKeyWord(message,typeOfPointss));
                    break;
                case "connect":
                    connectReceive();
                    break;
                case "startPoint":
                    startPoint = (WayPoint) JSONUtils.getObjectWithKeyWord(message, WayPoint.class);
                    Log.logInfo("SIMKERNEL", "Startpoint set to " + startPoint.getX() + "," + startPoint.getY() + "," + startPoint.getZ() + "," + startPoint.getW() + ".");
                    currentPosition = new Point(startPoint.getX(),startPoint.getY(),startPoint.getZ(),startPoint.getW());
                    break;
                case "currentMap":
                    map = (Map) JSONUtils.getObjectWithKeyWord(message, Map.class);
                    Log.logInfo("SIMKERNEL", "Map set to '" + map.getName() + "'.");
                    break;
                case "nextWayPoint":
                    Type typeOfWayPoint = new TypeToken<WayPoint>() {}.getType();
                    jobRequest((WayPoint) JSONUtils.getObjectWithKeyWord(message,typeOfWayPoint));
                    break;
                default:
                    Log.logWarning("SIMKERNEL", "No matching keyword when parsing JSON from Sockets. Data: " + message);
                    break;
            }
        }
        return null;
    }

    private void connectReceive() {
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
        connected = true;
        Log.logInfo("SIMKERNEL", "Connected to Core.");
    }

    private void wayPointReached() {
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("arrivedWaypoint"));
        connected = true;
        Log.logInfo("SIMKERNEL", "Arrived at waypoint. Waiting for next order.");
    }

    private void jobRequest(WayPoint nextPoint){
        Log.logInfo("SIMKERNEL", "Job request to drive to " + nextPoint.getX() + "," + nextPoint.getY() + "," + nextPoint.getZ() + "," + nextPoint.getW() + ".");
        Cost cost = new Cost(false,5,5,(long)0);
        if(!debugWithoutRosServer){
            List<Point> points = new ArrayList<>();
            points.add(currentPosition);
            points.add(currentPosition);
            points.add(nextPoint);
            Type typeOfCost = new TypeToken<Cost>() {}.getType();
            cost = (Cost) JSONUtils.getObjectWithKeyWord(restUtils.getJSONPostJSON("calcWeight",JSONUtils.arrayToJSONString(points)), typeOfCost);
        }
        Log.logInfo("SIMKERNEL", "Travel time to destination is " + cost.getWeight() + "s.");
        for(int i = 0;i <= 10 ; i++){
            try {
                Thread.sleep((cost.getWeight()*1000)/10);
                Location location = new Location(0,0,0,i*10);
                tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("percentage",location));
                Log.logInfo("SIMKERNEL", "travelled " + i*10 + "% of total route.");
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        wayPointReached();
        currentPosition = new Point(nextPoint.getX(),nextPoint.getY(),nextPoint.getZ(),nextPoint.getW());
    }

    private void calculateCost(ArrayList<Point> pointsTemp){
        ArrayList<Point> points = new ArrayList<>();
        points.add(currentPosition);
        points.addAll(pointsTemp);
        Log.logInfo("SIMKERNEL", "Cost request received. Requesting calculation from ROS Server.");
        Cost cost = new Cost(false,5,5,(long)0);
        if(!debugWithoutRosServer){
            Type typeOfCost = new TypeToken<Cost>() {}.getType();
            cost = (Cost) JSONUtils.getObjectWithKeyWord(restUtils.getJSONPostJSON("calcWeight",JSONUtils.arrayToJSONString(points)), typeOfCost);
        }
        Log.logInfo("SIMKERNEL", "Calculated cost between current and start: " + cost.getWeightToStart() + "s. Cost to end : " + cost.getWeight() + "s.");
        tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("cost",cost));
    }

    private void calculateTiming(ArrayList<Point> pointsTemp){
        ArrayList<Point> points = new ArrayList<>();
        points.add(currentPosition);
        points.addAll(pointsTemp);
        Log.logInfo("SIMKERNEL", "Timing request received. Requesting calculation from ROS Server.");
        Cost cost = new Cost(false,5,5,(long)0);
        if(!debugWithoutRosServer){
            Type typeOfCost = new TypeToken<Cost>() {}.getType();
            cost = (Cost) JSONUtils.getObjectWithKeyWord(restUtils.getJSONPostJSON("calcWeight",JSONUtils.arrayToJSONString(points)), typeOfCost);
        }
        Log.logInfo("SIMKERNEL", "Calculated timing between current and start: " + cost.getWeightToStart() + "s. Timing to end : " + cost.getWeight() + "s.");
        tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("costtiming",cost));
    }

    public static void main(String[] args) throws Exception {

        if (args.length != 2) {
            System.out.println("Need 2 arguments to run. Possible arguments: tcpclientport(int) tcpserverport(int)");
            System.exit(0);
        } else if (args.length == 2) {
            if (!args[0].isEmpty()) serverPort  = Integer.parseInt(args[0]);
            if (!args[1].isEmpty()) clientPort = Integer.parseInt(args[1]);
        }
        final SimKernel simKernel = new SimKernel(serverPort,clientPort);
    }
}

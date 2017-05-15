package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;
import org.eclipse.paho.client.mqttv3.MqttException;
import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.*;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener, TCPListener {

    private boolean debugWithoutBackEnd = true; // debug parameter to stop attempts to send or recieve messages from backbone.
    private static Log log;
    Level level = Level.CONFIG;
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final String wayPointFolder = "wayPoints";
    private final String restURLMAAS = "http://localhost:8080/";
    private final String restURLBackBone = "http://localhost:8080/";
    private final String socketAddress = "localhost";
    private final int serverPort = 5005;
    private final int clientPort = 5006;

    private MQTTUtils mqttUtils;
    private RESTUtils restUtilsMAAS;
    private RESTUtils restUtilsBackBone;
    private TCPUtils tcpUtils;

    private static HashMap<Integer, WayPoint> wayPoints = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, Vehicle> vehicles = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, SimulatedVehicle> simulatedVehicles = new HashMap<>();
    private static String currentMap;

    public Manager() {

    }

    public Manager(String currentMap) throws MqttException {
        log = new Log(this.getClass(), level);
        Manager.currentMap = currentMap;
        restUtilsMAAS = new RESTUtils(restURLMAAS);
        restUtilsBackBone = new RESTUtils(restURLBackBone);
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/#");
        tcpUtils = new TCPUtils(socketAddress,clientPort,serverPort,this);
        wayPoints = XMLUtils.loadWaypoints(wayPointFolder);
    }

    @Override
    public void parseMQTT(String topic, String message) {
        if (topic.matches("racecar/task")) {
            String[] waypointStringValues = message.split(" ");
            long[] waypointValues = new long[waypointStringValues.length];
            for (int index = 0; index < waypointStringValues.length; index++) {
                waypointValues[index] = Long.parseLong(waypointStringValues[index]);
            }
            long ID = waypointValues[0];
            if (vehicles.containsKey(ID)) {
                if (!vehicles.get(ID).getOccupied()) {
                    int n = waypointValues.length - 1;
                    long[] newArray = new long[n];
                    System.arraycopy(waypointValues, 1, newArray, 0, n);
                    jobSend(ID, newArray);
                } else {
                    Log.logWarning("MANAGER", "Vehicle with ID " + ID + " is occupied. Cant send route job.");
                }

            } else {
                Log.logWarning("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant send route job.");
            }
        } else if(topic.matches("racecar/tcptest")){
            parseTCP(message);
        } else if (topic.matches("racecar/[0-9]+/location")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                locationUpdate(ID, message);
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant set location.");
            }
        } else if (topic.matches("racecar/[0-9]+/route")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                routeUpdate(ID, message);
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant update route information.");
            }
        } else if (topic.matches("racecar/[0-9]+/waypoint")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                waypointUpdate(ID, message);
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant update route information.");
            }
        } else if (topic.matches("racecar/[0-9]+/kill")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                killVehicle(ID);
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cannot kill.");
            }
        }
    }

    private void waypointUpdate(long ID, String message) {
        vehicles.get(ID).setLastWayPoint(Integer.parseInt(message));
        Log.logInfo("MANAGER", "Vehicle with ID " + ID + " has reached waypoint " + message + ".");
    }

    private void locationUpdate(long ID, String message) {
        Type typeOfPoint = new TypeToken<Point>() {}.getType();
        vehicles.get(ID).setPoint((Point) JSONUtils.getObject(message,typeOfPoint));
        Log.logInfo("MANAGER", "Location update of vehicle with ID " + ID + ".");
    }

    private void routeUpdate(long ID, String message) {
        switch (message) {
            case "done":
                vehicles.get(ID).setOccupied(false);
                if (!debugWithoutBackEnd) {
                    restUtilsMAAS.getTextPlain("completeJob/" + ID);
                }
                Log.logInfo("MANAGER", "Vehicle with ID " + ID + " has completed his route.");
                break;
            case "error":
                vehicles.get(ID).setOccupied(false);
                Log.logInfo("MANAGER", "Vehicle with ID " + ID + " had errors in his route request.");
                break;
            case "notcomplete":
                vehicles.get(ID).setOccupied(true);
                Log.logInfo("MANAGER", "Vehicle with ID " + ID + " didn't complete his route yet.");
                break;
        }
    }

    private void jobSend(long ID, long[] waypointValues) {
        vehicles.get(ID).setOccupied(true);
        String message = Arrays.toString(waypointValues).replace(", ", " ").replace("[", "").replace("]", "").trim();
        Log.logInfo("MANAGER", "Route job send to vehicle with ID " + ID + " with wayPoints " + message);
        mqttUtils.publishMessage("racecar/" + ID + "/job", message);
    }

    @GET
    @Path("register")
    @Produces("text/plain")
    public Response register(@DefaultValue("1") @QueryParam("startwaypoint") int startwaypoint, @Context HttpServletResponse response) throws IOException {
        if (!wayPoints.containsKey(startwaypoint)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "Waypoint " + startwaypoint + " not found");
        } else {
            long id;
            if (!debugWithoutBackEnd) {
                id = Long.parseLong(restUtilsBackBone.getTextPlain("bot/newBot/car"));
            } else {
                id = (long) vehicles.size();
            }
            vehicles.put(id, new Vehicle(id, startwaypoint, wayPoints.get(startwaypoint)));
            Log.logInfo("MANAGER", "New vehicle registered. Given ID " + id + ". Has starting waypoint " + startwaypoint + ".");

            return Response.status(Response.Status.OK).
                    entity(id).
                    type("text/plain").
                    build();
        }
        return null;
    }


    @GET
    @Path("posAll")
    @Produces("application/json")
    public Response getPositions(@Context HttpServletResponse response) throws IOException {
        if (vehicles.isEmpty()) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "no vehicles registered yet");
        }else{
            List<Location> locations = new ArrayList<>();
            for (Vehicle vehicle : vehicles.values()) {
                locations.add(new Location(vehicle.getID(),vehicle.getLastWayPoint()));
            }
            Log.logInfo("MANAGER", "All vehicle Locations request has been completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(locations)).
                    type("application/json").
                    build();
        }
        return null;
    }

    @GET
    @Path("calcWeight/{start}/to/{stop}")
    @Produces("application/json")
    public Response calculateCostsRequest(@PathParam("start") final int startId, @PathParam("stop") final int endId, @Context HttpServletResponse response) throws IOException {
        if (!wayPoints.containsKey(startId) || !wayPoints.containsKey(endId)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "start or end waypoint not found");
        } else if (vehicles.isEmpty()) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "no vehicles registered yet");
        } else {
            List<Cost> costs = new ArrayList<>();
            for (Vehicle vehicle : vehicles.values()) {
                costs.add(new Cost(vehicle.getOccupied(), calculateCost(vehicle.getLocation(), wayPoints.get(startId)), calculateCost(wayPoints.get(startId), wayPoints.get(endId)), vehicle.getID()));
            }
            Log.logInfo("MANAGER", "Cost calculation request completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(costs)).
                    type("application/json").
                    build();
        }
        return null;
    }

    private long calculateCost(Point startPoint, Point endPoint){
        return (long)5;
    }


    @GET
    @Path("getmapname")
    @Produces("text/plain")
    public String getMapName() {
        return currentMap;
    }

    @GET
    @Path("getwaypoints")
    @Produces("application/json")
    public String getWayPoints() {
        return JSONUtils.objectToJSONStringWithKeyWord("wayPoints", wayPoints);
    }


    @GET
    @Path("getmappgm/{mapname}")
    @Produces("application/octet-stream")
    public Response getMapPGM(@PathParam("mapname") final String mapname, @Context HttpServletResponse response) {
        StreamingOutput fileStream = output -> {
            try {
                java.nio.file.Path path = Paths.get("maps/" + mapname + ".pgm");
                byte[] data = Files.readAllBytes(path);
                output.write(data);
                output.flush();
            } catch (Exception e) {
                response.sendError(HttpServletResponse.SC_NOT_FOUND, mapname + ".pgm not found");
            }
        };
        return Response
                .ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
                .header("content-disposition", "attachment; filename = " + mapname + ".pgm")
                .build();

    }

    @GET
    @Path("getmapyaml/{mapname}")
    @Produces("application/octet-stream")
    public Response getMapYAML(@PathParam("mapname") final String mapname, @Context HttpServletResponse response) {
        StreamingOutput fileStream = output -> {
            try {
                java.nio.file.Path path = Paths.get("maps/" + mapname + ".yaml");
                byte[] data = Files.readAllBytes(path);
                output.write(data);
                output.flush();
            } catch (Exception e) {
                response.sendError(HttpServletResponse.SC_NOT_FOUND, mapname + ".yaml not found");
            }
        };
        return Response
                .ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
                .header("content-disposition", "attachment; filename = " + mapname + ".yaml")
                .build();
    }


    public static void main(String[] args) throws Exception {
        if (args.length == 0) {
            System.out.println("Need at least 1 argument to run. Possible arguments: currentMap(String)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) {
                final Manager manager = new Manager(args[0]);
                new TomCatLauncher().start();
            }
        }
    }

    @Override
    public void parseTCP(String message) {
        if(message.matches("create\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(!simulatedVehicles.containsKey(simulationID)){
                simulatedVehicles.put(simulationID,new SimulatedVehicle((long)-1,simulationID,-1,null));
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("ACK");
                Log.logInfo("MANAGER", "New simulated vehicle registered with simulation ID " + simulationID + ".");
            }else{
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
                Log.logConfig("MANAGER","Cannot create vehicle with simulation ID " + simulationID + ". It already exists.");
            }
        }else if(message.matches("run\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                if(simulatedVehicles.get(simulationID).getID() == -1){
                    if(simulatedVehicles.get(simulationID).getLastWayPoint() != -1){
                        Long ID;
                        if (!debugWithoutBackEnd) {
                            ID = Long.parseLong(restUtilsBackBone.getTextPlain("bot/newBot/car"));
                        } else {
                            ID = (long) vehicles.size();
                        }
                        simulatedVehicles.get(simulationID).setID(ID);
                        vehicles.put(ID,simulatedVehicles.get(simulationID));
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " started. Given ID " + ID + ".");
                    }else{
                        Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It didn't have a starting point set.");
                        if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
                    }
                }else{
                    Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It was already started.");
                    if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
                }
            }else{
                Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It does not exist.");
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
            }
            if (!debugWithoutBackEnd)tcpUtils.sendUpdate("ACK");
        }else if(message.matches("stop\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                stopVehicle(simulatedVehicles.get(simulationID).getID());
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("ACK");
            }else{
                Log.logConfig("MANAGER","Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
            }
        }else if(message.matches("kill\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                killVehicle(simulatedVehicles.get(simulationID).getID());
                simulatedVehicles.remove(simulationID);
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("ACK");
            }else{
                Log.logConfig("MANAGER","Cannot kill vehicle with simulation ID " + simulationID + ". It does not exist.");
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
            }
        }else if(message.matches("set\\s[0-9]+\\s\\w+\\s\\w+")){
            String[] splitString = message.split("\\s+");
            Long simulationID = Long.parseLong(splitString[1]);
            if(simulatedVehicles.containsKey(simulationID)) {
                String parameter = splitString[2];
                String argument = splitString[3];
                switch (parameter) {
                    case "startpoint":
                        simulatedVehicles.get(simulationID).setLastWayPoint(Integer.parseInt(argument));
                        simulatedVehicles.get(simulationID).setLocation(wayPoints.get(Integer.parseInt(argument)));
                        if(simulatedVehicles.get(simulationID).getID() != -1){
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setLastWayPoint(Integer.parseInt(argument));
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setLocation(wayPoints.get(Integer.parseInt(argument)));
                        }
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given starting point ID " + argument + ".");
                        break;
                    case "speed":
                        simulatedVehicles.get(simulationID).setSpeed(Float.parseFloat(argument));
                        if(simulatedVehicles.get(simulationID).getID() != -1){
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setSpeed(Float.parseFloat(argument));
                        }
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given speed " + argument + ".");
                        break;
                    case "name":
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given name " + argument + ".");
                        break;
                }
            }else{
                Log.logConfig("MANAGER","Cannot change vehicle with simulation ID " + simulationID + ". It does not exist.");
                if (!debugWithoutBackEnd)tcpUtils.sendUpdate("NACK");
            }
        }
    }

    private void stopVehicle(Long ID){
        mqttUtils.publishMessage("racecar/" + ID + "/job","stop");
        Log.logInfo("MANAGER", "Vehicle with ID " + ID +  " Stopped.");
    }

    private void killVehicle(Long ID){
        vehicles.remove(ID);
        Log.logInfo("MANAGER", "Vehicle with ID " + ID +  " killed.");
    }
}

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
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener, TCPListener {

    private boolean debugWithoutBackBone = true; // debug parameter to stop attempts to send or recieve messages from backbone.
    private boolean debugWithoutMAAS = true; // debug parameter to stop attempts to send or recieve messages from MAAS
    private static Log log;
    Level level = Level.CONFIG;
    private final String mqttBroker = "tcp://143.129.39.151:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final String wayPointFolder = "wayPoints";
    private final String restURLMAAS = "http://localhost:8080/";
    private final String restURLBackBone = "http://146.175.140.44:1994/";
    private final int serverPort = 5007;
    private final int clientPort = 5007;

    private static MQTTUtils mqttUtils;
    private static RESTUtils restUtilsMAAS;
    private static RESTUtils restUtilsBackBone;
    private static TCPUtils tcpUtils;

    private static HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, Vehicle> vehicles = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, SimulatedVehicle> simulatedVehicles = new HashMap<>();
    private static String currentMap;
    private static ArrayList<Cost> costs = new ArrayList<>();

    public Manager() {

    }

    public Manager(String currentMap) throws MqttException, IOException {
        log = new Log(this.getClass(), level);
        Manager.currentMap = currentMap;
        restUtilsMAAS = new RESTUtils(restURLMAAS);
        restUtilsBackBone = new RESTUtils(restURLBackBone);
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/#");
        wayPoints = XMLUtils.loadWaypoints(wayPointFolder);
        tcpUtils = new TCPUtils(5005,this);
        tcpUtils.start();
    }

    @Override
    public void parseMQTT(String topic, String message) {
        if(topic.matches("racecar/tcptest")){
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
        }else if (topic.matches("racecar/[0-9]+/costanswer")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Type typeOfCost = new TypeToken<Cost>() {}.getType();
                costs.add((Cost) JSONUtils.getObject(message,typeOfCost));
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cannot kill.");
            }
        }
    }

    private void waypointUpdate(long ID, String message) {
        vehicles.get(ID).setLastWayPoint(Long.parseLong(message));
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
                if (!debugWithoutMAAS) {
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

    @GET
    @Path("register")
    @Produces("text/plain")
    public Response register(@DefaultValue("1") @QueryParam("startwaypoint") long startWayPoint, @Context HttpServletResponse response) throws IOException {
        if (!wayPoints.containsKey(startWayPoint)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "Waypoint " + startWayPoint + " not found");
        } else {
            long id;
            if (!debugWithoutBackBone) {
                id = Long.parseLong(restUtilsBackBone.getJSON("bot/newBot/car"));
            } else {
                id = (long) vehicles.size();
            }
            vehicles.put(id, new Vehicle(id, startWayPoint, wayPoints.get(startWayPoint)));
            Log.logInfo("MANAGER", "New vehicle registered. Given ID " + id + ". Has starting waypoint " + startWayPoint + ".");

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
    public Response calculateCostsRequest(@PathParam("start") final long startId, @PathParam("stop") final long endId, @Context HttpServletResponse response) throws IOException, InterruptedException {
        if (!wayPoints.containsKey(startId) || !wayPoints.containsKey(endId)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "start or end waypoint not found");
        } else if (vehicles.isEmpty()) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "no vehicles registered yet");
        } else {
            for (Vehicle vehicle : vehicles.values()) {
                mqttUtils.publishMessage("racecar/" + Long.toString(vehicle.getID()) + "/costrequest", Long.toString(startId) + " " + Long.toString(endId));
            }
            while(costs.size() != getRunningVehicleAmount()){
                Log.logInfo("MANAGER", "waiting for vehicles to complete request.");
                Thread.sleep(1000);
            }
            ArrayList<Cost> costCopy = (ArrayList<Cost>)costs.clone();
            this.costs.clear();
            Log.logInfo("MANAGER", "Cost calculation request completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(costCopy)).
                    type("application/json").
                    build();
        }
        return null;
    }

    private int getRunningVehicleAmount(){
        int simulated = 0;
        for(Vehicle vehicle : vehicles.values()){
            if(vehicle.getClass() == SimulatedVehicle.class){
                simulated++;
            }
        }
        return vehicles.size()-simulated;
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

    @POST
    @Path("executeJob")
    @Consumes("application/json")
    public Response jobRequest(String data,@Context HttpServletResponse response) throws IOException {
        Type typeOfJob = new TypeToken<Job>() {}.getType();
        Job job = (Job) JSONUtils.getObject(data,typeOfJob);

        if (vehicles.containsKey(job.getIdVehicle())) {
            if (!vehicles.get(job.getIdVehicle()).getOccupied()) {
                if(wayPoints.containsKey(job.getIdStart()) && wayPoints.containsKey(job.getIdEnd())){
                    jobSend(job.getIdVehicle(),job.getIdStart(),job.getIdEnd());
                    return Response.status(Response.Status.OK).build();
                }else{
                    response.sendError(HttpServletResponse.SC_NOT_FOUND, "waypoints " + job.getIdStart() + " and/or " + job.getIdEnd() + "not found.");
                    Log.logWarning("MANAGER", "Can't send route job as waypoints " + job.getIdStart() + " and/or " + job.getIdEnd() + " were not found.");
                }

            } else {
                response.sendError(HttpServletResponse.SC_NOT_FOUND, "Vehicle with ID " + job.getIdVehicle() + " is occupied.");
                Log.logWarning("MANAGER", "Vehicle with ID " + job.getIdVehicle() + " is occupied. Cant send route job.");
            }

        } else {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "Vehicle with ID " + job.getIdVehicle() + " not found.");
            Log.logWarning("MANAGER", "Vehicle with ID " + job.getIdVehicle() + " doesn't exist. Cant send route job.");
        }
        return Response.status(Response.Status.OK).build();
    }

    private void jobSend(long ID, long startID, long endID) {
        vehicles.get(ID).setOccupied(true);
        Log.logInfo("MANAGER", "Route job send to vehicle with ID " + ID + " from " + startID + " to " + endID + ".");
        mqttUtils.publishMessage("racecar/" + ID + "/job", Long.toString(startID) + " " + Long.toString(endID));
    }

    @Override
    public String parseTCP(String message) {
        if(message.matches("create\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(!simulatedVehicles.containsKey(simulationID)){
                simulatedVehicles.put(simulationID,new SimulatedVehicle((long)-1,simulationID,-1,null));
                Log.logInfo("MANAGER", "New simulated vehicle registered with simulation ID " + simulationID + ".");
                return "ACK";
            }else{
                Log.logConfig("MANAGER","Cannot create vehicle with simulation ID " + simulationID + ". It already exists.");
                return "NACK";
            }
        }else if(message.matches("run\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                if(simulatedVehicles.get(simulationID).getID() == -1){
                    if(simulatedVehicles.get(simulationID).getLastWayPoint() != -1){
                        Long ID;
                        if (!debugWithoutBackBone) {
                            ID = Long.parseLong(restUtilsBackBone.getJSON("bot/newBot/car"));
                        } else {
                            ID = (long) vehicles.size();
                        }
                        simulatedVehicles.get(simulationID).setID(ID);
                        vehicles.put(ID,simulatedVehicles.get(simulationID));
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " started. Given ID " + ID + ".");
                        return "ACK";
                    }else{
                        Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It didn't have a starting point set.");
                        return "NACK";
                    }
                }else{
                    Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It was already started.");
                    return "NACK";
                }
            }else{
                Log.logConfig("MANAGER","Cannot start vehicle with simulation ID " + simulationID + ". It does not exist.");
                return "NACK";
            }
        }else if(message.matches("stop\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                stopVehicle(simulatedVehicles.get(simulationID).getID());
                return "ACK";
            }else{
                Log.logConfig("MANAGER","Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
                return "NACK";
            }
        }else if(message.matches("kill\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                killVehicle(simulatedVehicles.get(simulationID).getID());
                simulatedVehicles.remove(simulationID);
                return "ACK";
            }else{
                Log.logConfig("MANAGER","Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
                return "NACK";
            }
        }else if(message.matches("restart\\s[0-9]+")){
            long simulationID = Long.parseLong(message.replaceAll("\\D+", ""));
            if(simulatedVehicles.containsKey(simulationID)){
                if(simulatedVehicles.get(simulationID).getLastWayPoint() != -1) {
                    simulatedVehicles.get(simulationID).setLastWayPoint(simulatedVehicles.get(simulationID).getStartPoint());
                    simulatedVehicles.get(simulationID).setLocation(wayPoints.get(simulatedVehicles.get(simulationID).getStartPoint()));
                    if (simulatedVehicles.get(simulationID).getID() != -1) {
                        vehicles.get(simulatedVehicles.get(simulationID).getID()).setLastWayPoint(simulatedVehicles.get(simulationID).getStartPoint());
                        vehicles.get(simulatedVehicles.get(simulationID).getID()).setLocation(wayPoints.get(simulatedVehicles.get(simulationID).getStartPoint()));
                    }
                    return "ACK";
                }else{
                    Log.logConfig("MANAGER","Cannot restart vehicle with simulation ID " + simulationID + ". It isn't started.");
                    return "NACK";
                }
            }else{
                Log.logConfig("MANAGER","Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
                return "NACK";
            }
        }else if(message.matches("set\\s[0-9]+\\s\\w+\\s\\w+")){
            String[] splitString = message.split("\\s+");
            Long simulationID = Long.parseLong(splitString[1]);
            if(simulatedVehicles.containsKey(simulationID)) {
                String parameter = splitString[2];
                String argument = splitString[3];
                switch (parameter) {
                    case "startpoint":
                        simulatedVehicles.get(simulationID).setLastWayPoint(Long.parseLong(argument));
                        simulatedVehicles.get(simulationID).setLocation(wayPoints.get(Long.parseLong(argument)));
                        if(simulatedVehicles.get(simulationID).getID() != -1){
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setLastWayPoint(Long.parseLong(argument));
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setLocation(wayPoints.get(Long.parseLong(argument)));
                        }
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given starting point ID " + argument + ".");
                        return "ACK";
                    case "speed":
                        simulatedVehicles.get(simulationID).setSpeed(Float.parseFloat(argument));
                        if(simulatedVehicles.get(simulationID).getID() != -1){
                            vehicles.get(simulatedVehicles.get(simulationID).getID()).setSpeed(Float.parseFloat(argument));
                        }
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given speed " + argument + ".");
                        return "ACK";
                    case "name":
                        Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID +  " given name " + argument + ".");
                        return "ACK";
                }
            }else{
                Log.logConfig("MANAGER","Cannot change vehicle with simulation ID " + simulationID + ". It does not exist.");
                return "NACK";
            }
        }
        return "NACK";
    }

    private void stopVehicle(Long ID){
        mqttUtils.publishMessage("racecar/" + ID + "/job","stop");
        Log.logInfo("MANAGER", "Vehicle with ID " + ID +  " Stopped.");
    }

    private void killVehicle(Long ID){
        vehicles.remove(ID);
        Log.logInfo("MANAGER", "Vehicle with ID " + ID +  " killed.");
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
}

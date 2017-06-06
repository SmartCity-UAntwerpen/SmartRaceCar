package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;
import org.eclipse.paho.client.mqttv3.MqttException;

import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener {

    private boolean debugWithoutBackBone = true; // debug parameter to stop attempts to send or recieve messages from backbone.
    private boolean debugWithoutMAAS = true; // debug parameter to stop attempts to send or recieve messages from MAAS
    private static Log log;
    Level level = Level.INFO;
    private final String mqttBroker = "tcp://143.129.39.151:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final String restURLMAAS = "http://localhost:8080/";
    private final String restURLBackBone = "http://143.129.39.151:10000/";

    private static MQTTUtils mqttUtils;
    private static RESTUtils restUtilsMAAS;
    private static RESTUtils restUtilsBackBone;


    private static HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, Vehicle> vehicles = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static String currentMap;
    private static String mapsPath;
    private static ArrayList<Cost> costs = new ArrayList<>();

    public Manager() {

    }

    public Manager(String currentMap, String mapsPath) throws MqttException, IOException {
        log = new Log(this.getClass(), level);
        Manager.currentMap = currentMap;
        Manager.mapsPath = mapsPath;
        log.logConfig("MANAGER", "Startup parameters: Map: " + currentMap + " | Path to maps folder: " + mapsPath);

        restUtilsMAAS = new RESTUtils(restURLMAAS);
        restUtilsBackBone = new RESTUtils(restURLBackBone);
        loadWayPoints();
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/#");
    }

    private void loadWayPoints() {
        if (debugWithoutBackBone) {
            wayPoints.put((long) 8, new WayPoint(8, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
            wayPoints.put((long) 9, new WayPoint(9, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
            wayPoints.put((long) 10, new WayPoint(10, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
            wayPoints.put((long) 11, new WayPoint(11, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
        } else {
            String jsonString = restUtilsBackBone.getJSON("map/stringmapjson/car");
            JSONUtils.isJSONValid(jsonString);
            Type typeOfWayPointArray = new TypeToken<ArrayList<WayPoint>>() {
            }.getType();
            ArrayList<WayPoint> wayPointsTemp = (ArrayList<WayPoint>) JSONUtils.getObject(jsonString, typeOfWayPointArray);
            for (WayPoint wayPoint : wayPointsTemp) {
                wayPoints.put(wayPoint.getID(), wayPoint);
                Log.logConfig("MANAGER", "Added wayPoint with ID " + wayPoint.getID() + " and coordinates " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW() + ".");
            }
        }
        Log.logInfo("CORE", "All possible waypoints(" + wayPoints.size() + ") received.");
    }

    @Override
    public void parseMQTT(String topic, String message) {
        if (topic.matches("racecar/[0-9]+/route")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                routeUpdate(ID, message);
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant update route information.");
            }
        } else if (topic.matches("racecar/[0-9]+/percentage")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Type typeOfLocation = new TypeToken<Location>() {
                }.getType();
                Location location = (Location) JSONUtils.getObject(message, typeOfLocation);
                vehicles.get(ID).getLocation().setPercentage(location.getPercentage());
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant update route percentage.");
            }
        } else if (topic.matches("racecar/[0-9]+/costanswer")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Type typeOfCost = new TypeToken<Cost>() {
                }.getType();
                costs.add((Cost) JSONUtils.getObject(message, typeOfCost));
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cannot process cost answer.");
            }
        } else if (topic.matches("racecar/[0-9]+/available")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                vehicles.get(ID).setAvailable(Boolean.parseBoolean(message));
                if (vehicles.get(ID).isAvailable())
                    Log.logInfo("MANAGER", "Vehicle with ID " + ID + " set to be available.");
                else
                    Log.logInfo("MANAGER", "Vehicle with ID " + ID + " set to be no longer available.");
            } else {
                Log.logConfig("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cannot set availability.");
            }
        }
    }

    private void routeUpdate(long ID, String message) {
        switch (message) {
            case "done":
                vehicles.get(ID).setOccupied(false);
                if (!debugWithoutMAAS) {
                    restUtilsMAAS.getTextPlain("completeJob/" + ID);
                }
                vehicles.get(ID).getLocation().setIdStart(vehicles.get(ID).getLocation().getIdEnd());
                vehicles.get(ID).getLocation().setPercentage(0);
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
    @Path("register/{startwaypoint}")
    @Produces("text/plain")
    public Response register(@PathParam("startwaypoint") long startWayPoint, @Context HttpServletResponse response) throws IOException {
        if (!wayPoints.containsKey(startWayPoint)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "Waypoint " + startWayPoint + " not found");
        } else {
            long id;
            if (!debugWithoutBackBone) {
                id = Long.parseLong(restUtilsBackBone.getJSON("bot/newBot/car"));
            } else {
                id = (long) vehicles.size();
            }
            vehicles.put(id, new Vehicle(id, startWayPoint));
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
        } else {
            List<Location> locations = new ArrayList<>();
            for (Vehicle vehicle : vehicles.values()) {
                if (vehicle.isAvailable()) {
                    locations.add(vehicle.getLocation());
                }
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
    @Path("calcWeight/{idStart}/{idStop}")
    @Produces("application/json")
    public Response calculateCostsRequest(@PathParam("idStart") long startId, @PathParam("idStop") long endId, @Context HttpServletResponse response) throws IOException, InterruptedException {
        if (!wayPoints.containsKey(startId) || !wayPoints.containsKey(endId)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "start or end waypoint not found");
        } else if (vehicles.isEmpty()) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "no vehicles registered yet");
        } else {
            int totalVehicles = 0;
            for (Vehicle vehicle : vehicles.values()) {
                if (vehicle.isAvailable()) {
                    totalVehicles++;
                    mqttUtils.publishMessage("racecar/" + Long.toString(vehicle.getID()) + "/costrequest", Long.toString(startId) + " " + Long.toString(endId));
                }
            }

            while (costs.size() != totalVehicles) {
                Log.logInfo("MANAGER", "waiting for vehicles to complete request.");
                Thread.sleep(200);
            }
            ArrayList<Cost> costCopy = (ArrayList<Cost>) costs.clone();
            this.costs.clear();
            Log.logInfo("MANAGER", "Cost calculation request completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(costCopy)).
                    type("application/json").
                    build();
        }
        return null;
    }

    @GET
    @Path("getmapname")
    @Produces("text/plain")
    public String getMapName() {
        return currentMap;
    }

    @GET
    @Path("delete/{id}")
    @Produces("text/plain")
    public Response deleteVehicle(@PathParam("id") final long id, @Context HttpServletResponse response) throws IOException {
        if (vehicles.containsKey(id)) {
            if (!debugWithoutBackBone) restUtilsBackBone.getTextPlain("delete/" + id);
            vehicles.remove(id);
            Log.logInfo("MANAGER", "Vehicle with ID " + id + " stopped. ID removed.");
            return Response.status(Response.Status.OK).build();
        } else {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, " vehicle with id " + id + "  not found");
        }
        return null;
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
    public Response getMapPGM(@PathParam("mapname") final String mapname, @Context HttpServletResponse response) throws UnsupportedEncodingException {
        StreamingOutput fileStream = output -> {
            try {
                java.nio.file.Path path = Paths.get(mapsPath + "/" + mapname + ".pgm");
                byte[] data = Files.readAllBytes(path);
                output.write(data);
                output.flush();
            } catch (Exception e) {
                log.logWarning("MANAGER", String.valueOf(e));
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
                java.nio.file.Path path = Paths.get(mapsPath + "/" + mapname + ".yaml");
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

    @GET
    @Path("executeJob/{idJob}/{idVehicle}/{idStart}/{idEnd}")
    @Produces("text/plain")
    public Response jobRequest(@PathParam("idJob") long idJob, @PathParam("idVehicle") long idVehicle, @PathParam("idStart") long idStart, @PathParam("idEnd") long idEnd, String data, @Context HttpServletResponse response) throws IOException {
        Job job = new Job(idJob, idStart, idEnd, idVehicle);
        if (vehicles.containsKey(job.getIdVehicle())) {
            if (!vehicles.get(job.getIdVehicle()).getOccupied() && vehicles.get(job.getIdVehicle()).isAvailable()) {
                if (wayPoints.containsKey(job.getIdStart())) {
                    if (wayPoints.containsKey(job.getIdEnd())) {
                        Location location = vehicles.get(job.getIdVehicle()).getLocation();
                        location.setIdStart(job.getIdStart());
                        location.setIdEnd(job.getIdEnd());
                        location.setPercentage(0);
                        vehicles.get(job.getIdVehicle()).setLocation(location);
                        jobSend(job.getIdVehicle(), job.getIdStart(), job.getIdEnd());
                        return Response.status(Response.Status.OK).build();
                    } else {
                        response.sendError(HttpServletResponse.SC_NOT_FOUND,"No matching ending waypoint found");
                        Log.logWarning("MANAGER", "Can't send route job as waypoints " + job.getIdStart() + " was not found.");
                    }
                } else {
                    response.sendError(HttpServletResponse.SC_NOT_FOUND,"No matching starting waypoint found");
                    Log.logWarning("MANAGER", "Can't send route job as waypoints " + job.getIdStart() + " was not found.");
                }
            } else {
                response.sendError(HttpServletResponse.SC_FORBIDDEN, "This vehicle is busy or occupied");
                Log.logWarning("MANAGER", "Vehicle with ID " + job.getIdVehicle() + " is occupied or not available. Cant send route job.");
            }

        } else {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "This vehicle doesn't exist");
            Log.logWarning("MANAGER", "Vehicle with ID " + job.getIdVehicle() + " doesn't exist. Cant send route job.");
        }
        return Response.status(Response.Status.OK).build();
    }

    private void jobSend(long ID, long startID, long endID) {
        vehicles.get(ID).setOccupied(true);
        Log.logInfo("MANAGER", "Route job send to vehicle with ID " + ID + " from " + startID + " to " + endID + ".");
        mqttUtils.publishMessage("racecar/" + ID + "/job", Long.toString(startID) + " " + Long.toString(endID));
    }

    public static void main(String[] args) throws Exception {
        if (args.length != 2) {
            System.out.println("Need 2 arguments to run. Possible arguments: currentMap(String) mapsPath(String)");
            System.exit(0);
        } else if (args.length == 2) {
            final Manager manager = new Manager(args[0], args[1]);
            new TomCatLauncher().start();

        }
    }
}

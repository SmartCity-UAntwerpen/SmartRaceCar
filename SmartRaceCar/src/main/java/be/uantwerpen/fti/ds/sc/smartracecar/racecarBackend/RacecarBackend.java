package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import be.uantwerpen.fti.ds.sc.smartracecar.core.HeartbeatPublisher;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;

import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.*;
import java.lang.reflect.Type;
import java.net.URLDecoder;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.logging.Level;

/**
 * Module representing the management or dispatching module of the F1 service.
 */
@Path("carmanager")
public class RacecarBackend implements MQTTListener {

    //Standard settings (without config file loaded)
    private static boolean debugWithoutBackBone = false; // debug parameter to stop attempts to send or recieve messages from backbone.
    private static boolean debugWithoutMAAS = false; // debug parameter to stop attempts to send or recieve messages from MAAS
    private String mqttBroker = "tcp://smartcity.ddns.net:1883"; // MQTT Broker URL
    private String mqqtUsername = "root"; // MQTT Broker Username
    private String mqttPassword = "smartcity"; // MQTT Broker Password
    private static String restURLMAAS = "http://smartcity.ddns.net:8090"; // REST Service URL to MAAS
    private static String restURLBackBone = "http://smartcity.ddns.net:10000";// REST Service URL to BackBone.

    //Help services
    private static MQTTUtils mqttUtils;
    private static RESTUtils restUtilsMAAS;
    private static RESTUtils restUtilsBackBone;
    private static HeartbeatChecker heartbeatChecker;

    //variables
    private Log log; // logging instance
    private static HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // Hashmap of all vehicles mapped by ID.
    private static HashMap<Long, Vehicle> vehicles = new HashMap<>(); // Hashmap of all vehicles mapped by ID.
    private static String currentMap = "gangV"; // Information on the currently used map.
    private static String mapsPath = ".\\release\\maps"; //"C:\\maps"; // Path to the location of the maps.xml file where maps are stored.
    private static ArrayList<Cost> costs = new ArrayList<>(); // Contains all currently received calculated costs when a cost request was made.

    /**
     * Module representing the management or dispatching module of the F1 service. Empty constructor used by REST.
     */
    public RacecarBackend() {

    }

    /**
     * Module representing the management or dispatching module of the F1 service.
     *
     * @param start help parameter to make a distinction between the two constructors
     */
    public RacecarBackend(Boolean start) throws Exception {
        String asciiArt1 = FigletFont.convertOneLine("SmartCity");
        System.out.println(asciiArt1);
        System.out.println("-------------------------------------------------------------------");
        System.out.println("-------------------- F1 Racecar Backend - v1.0 --------------------");
        System.out.println("-------------------------------------------------------------------");
        startTomCatServer();
        loadConfig();
        Log.logConfig("RACECAR_BACKEND", "Startup parameters: Map: " + currentMap + " | Path to maps folder: " + mapsPath);
        restUtilsMAAS = new RESTUtils(restURLMAAS);
        restUtilsBackBone = new RESTUtils(restURLBackBone);
        loadWayPoints();
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/#");
        heartbeatChecker = new HeartbeatChecker(this);
        heartbeatChecker.start();
    }

    /**
     * Start the build-in TomCat Server.
     */
    private void startTomCatServer(){
        Thread tomcat = new Thread() {
            public void run() {
                try {
                    new TomCatLauncher().start();
                } catch(InterruptedException v) {
                    System.out.println(v);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        };
        tomcat.start();
    }

    /**
     * Help method to load all configuration parameters from the properties file with the same name as the class.
     * If it's not found then it will use the default ones.
     */
    @SuppressWarnings("Duplicates")
    private void loadConfig() {
        Properties prop = new Properties();
        InputStream input = null;
        try {
            String path = RacecarBackend.class.getProtectionDomain().getCodeSource().getLocation().getPath();
            String decodedPath = URLDecoder.decode(path, "UTF-8");
            decodedPath = decodedPath.replace("RacecarBackend.jar", "");
            input = new FileInputStream(decodedPath + "/racecarbackend.properties");
            prop.load(input);
            String debugLevel = prop.getProperty("debugLevel");
            switch (debugLevel) {
                case "debug":
                    log = new Log(this.getClass(), Level.CONFIG);
                    break;
                case "info":
                    log = new Log(this.getClass(), Level.INFO);
                    break;
                case "warning":
                    log = new Log(this.getClass(), Level.WARNING);
                    break;
                case "severe":
                    log = new Log(this.getClass(), Level.SEVERE);
                    break;
            }
            debugWithoutBackBone = Boolean.parseBoolean(prop.getProperty("debugWithoutBackBone"));
            debugWithoutMAAS = Boolean.parseBoolean(prop.getProperty("debugWithoutMAAS"));
            mqttBroker = "tcp://" + prop.getProperty("mqttBroker");
            mqqtUsername = prop.getProperty("mqqtUsername");
            mqttPassword = prop.getProperty("mqttPassword");
            restURLMAAS = prop.getProperty("restURLMAAS");
            restURLBackBone = prop.getProperty("restURLBackBone");
            currentMap = prop.getProperty("currentMap");
            mapsPath = prop.getProperty("mapsPath");
            Log.logInfo("RACECAR_BACKEND", "Config loaded");
        } catch (IOException ex) {
            log = new Log(this.getClass(), Level.CONFIG);
            Log.logWarning("RACECAR_BACKEND", "Could not read config file. Loading default settings. " + ex);
        } finally {
            if (input != null) {
                try {
                    input.close();
                } catch (IOException e) {
                    Log.logWarning("RACECAR_BACKEND", "Could not read config file. Loading default settings. " + e);
                }
            }
        }
    }

    /**
     * Request all possible waypoints from the BackBone through a REST Get request.
     */
    private void loadWayPoints() {
        wayPoints.clear();
        if (debugWithoutBackBone) { // Temp waypoints for when they can't be requested from back-end services.
            switch(currentMap) {
                case "zbuilding":
                    log.logConfig("RACECAR_BACKEND", "Loading waypoints for " + currentMap);
                    wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
                    wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
                    wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
                    wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
                    break;
                case "V314":
                    log.logConfig("RACECAR_BACKEND", "Loading waypoints for " + currentMap);
                    wayPoints.put((long) 46, new WayPoint(46, (float) -3.0, (float) -1.5, (float) 0.07, (float) 1.00));
                    wayPoints.put((long) 47, new WayPoint(47, (float) 1.10, (float) -1.20, (float) 0.07, (float) 1.00));
                    wayPoints.put((long) 48, new WayPoint(48, (float) 4.0, (float) -0.90, (float) -0.68, (float) 0.73));
                    wayPoints.put((long) 49, new WayPoint(49, (float) 4.54, (float) -4.49, (float) -0.60, (float) 0.80));
                    break;
                case "gangV":
                    log.logConfig("RACECAR_BACKEND", "Loading waypoints for " + currentMap);
                    wayPoints.put((long) 46, new WayPoint(46, (float) -6.1, (float) -28.78, (float) 0.73, (float) 0.69));
                    wayPoints.put((long) 47, new WayPoint(47, (float) -6.47, (float) -21.69, (float) 0.66, (float) 0.75));
                    wayPoints.put((long) 48, new WayPoint(48, (float) -5.91, (float) -1.03, (float) 0.52, (float) 0.85));
                    wayPoints.put((long) 49, new WayPoint(49, (float) 6.09, (float) 0.21, (float) -0.04, (float) 1.00));
                    break;
                default:
                    log.logWarning("RACECAR_BACKEND", "The backbone could not be reached and there were no default waypoints for this map");
                    wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
                    wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
                    wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
                    wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
            }
        } else {
            String jsonString = restUtilsBackBone.getJSON("map/stringmapjson/car"); //when the map is changed, another map needs to be manually loaded in the backbone database
            JSONUtils.isJSONValid(jsonString);
            Type typeOfWayPointArray = new TypeToken<ArrayList<WayPoint>>() {
            }.getType();
            ArrayList<WayPoint> wayPointsTemp = (ArrayList<WayPoint>) JSONUtils.getObject(jsonString, typeOfWayPointArray);
            for (WayPoint wayPoint : wayPointsTemp) {
                wayPoints.put(wayPoint.getID(), wayPoint);
                Log.logConfig("RACECAR_BACKEND", "Added wayPoint with ID " + wayPoint.getID() + " and coordinates " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW() + ".");
            }
        }
        Log.logInfo("RACECAR_BACKEND", "All possible waypoints(" + wayPoints.size() + ") received.");
    }

    /**
     * Interfaced method to parse MQTT message and topic after MQTT callback is triggered by incoming message.
     * Used by messages coming from all vehicles.
     *
     * @param topic   received MQTT topic
     * @param message received MQTT message string
     */
    @Override
    public void parseMQTT(String topic, String message) {
        if (topic.matches("racecar/[0-9]+/route")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                routeUpdate(ID, message);
            } else {
                Log.logConfig("RACECAR_BACKEND", "Vehicle with ID " + ID + " doesn't exist. Cant update route information.");
            }
        } else if (topic.matches("racecar/[0-9]+/percentage")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Type typeOfLocation = new TypeToken<Location>() {
                }.getType();
                Location location = (Location) JSONUtils.getObject(message, typeOfLocation);
                vehicles.get(ID).getLocation().setPercentage(location.getPercentage());
            } else {
                Log.logConfig("RACECAR_BACKEND", "Vehicle with ID " + ID + " doesn't exist. Cant update route percentage.");
            }
        } else if (topic.matches("racecar/[0-9]+/costanswer")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Type typeOfCost = new TypeToken<Cost>() {
                }.getType();
                costs.add((Cost) JSONUtils.getObject(message, typeOfCost));
            } else {
                Log.logConfig("RACECAR_BACKEND", "Vehicle with ID " + ID + " doesn't exist. Cannot process cost answer.");
            }
        } else if (topic.matches("racecar/[0-9]+/locationupdate")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " has it's location changed to waypoint " + message + ".");
                Location loc = vehicles.get(ID).getLocation();
                loc.setIdStart(Long.parseLong(message));
                loc.setIdEnd(Long.parseLong(message));
                vehicles.get(ID).setLocation(loc);
            } else {
                Log.logConfig("RACECAR_BACKEND", "Vehicle with ID " + ID + " doesn't exist. Cannot set new location.");
            }
        } else if (topic.matches("racecar/[0-9]+/available")) {
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            if (vehicles.containsKey(ID)) {
                vehicles.get(ID).setAvailable(Boolean.parseBoolean(message));
                if (vehicles.get(ID).isAvailable())
                    Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " set to be available.");
                else
                    Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " set to be no longer available.");
            } else {
                Log.logConfig("RACECAR_BACKEND", "Vehicle with ID " + ID + " doesn't exist. Cannot set availability.");
            }
        } else if (topic.matches("racecar/[0-9]+/heartbeat")) { //status can also be send along in the message part
            long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
            Date timestamp = new Date(); //time is allocated in the constructor with millisecond precision
            if(vehicles.containsKey(ID))
            {
                Vehicle vehicle = vehicles.get(ID);
                vehicle.setHeartbeat(timestamp);
                vehicles.put(ID,vehicle); //replaces the previous value
                log.logConfig("RACECAR_BACKEND", "Heartbeat of vehicle with ID " + ID + " updated to: " + timestamp);
            }
            else
                log.logConfig("RACECAR_BACKEND", "Heartbeat of vehicle with ID " + ID + "could not be updated as it does not exist");
        }
    }


    /**
     * Parses MQTT message received with specific route status update.
     *
     * @param ID      ID of the vehicle.
     * @param message Received MQTT message string to be parsed.
     */
    private void routeUpdate(long ID, String message) {
        switch (message) {
            case "done":
                vehicles.get(ID).setOccupied(false);
                if (!debugWithoutMAAS) {
                    restUtilsMAAS.getTextPlain("completeJob/" + vehicles.get(ID).getJob().getIdJob()); //TODO: deze methode verplaatsen naar de backbone
                }
                vehicles.get(ID).getLocation().setPercentage(100);
                Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " has completed his route.");
                break;
            case "error":
                vehicles.get(ID).setOccupied(false);
                Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " had errors in his route request.");
                break;
            case "notcomplete":
                vehicles.get(ID).setOccupied(true);
                Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + ID + " didn't complete his route yet.");
                break;
        }
    }

    /**
     * REST GET server service to register a vehicle. This in return does a REST GET request with the BackEnd service
     * to request and ID for the registring vehicle from them.
     *
     * @param startWayPoint Starting waypoint of the vehicle
     * @return REST response of the type Text Plain containing the ID.
     */
    @GET
    @Path("register/{startwaypoint}")
    @Produces("text/plain")
    public Response register(@PathParam("startwaypoint") long startWayPoint, @Context HttpServletResponse response) throws IOException {
        if (!wayPoints.containsKey(startWayPoint)) {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "Waypoint " + startWayPoint + " not found");
        } else {
            long id;
            if (debugWithoutBackBone) {
                id = (long) vehicles.size();

            } else {
                id = Long.parseLong(restUtilsBackBone.getJSON("bot/newBot/car"));
            }
            vehicles.put(id, new Vehicle(id, startWayPoint));
            Log.logInfo("RACECAR_BACKEND", "New vehicle registered. Given ID " + id + ". Has starting waypoint " + startWayPoint + ".");

            return Response.status(Response.Status.OK).
                    entity(id).
                    type("text/plain").
                    build();
        }
        return null;
    }


    /**
     * REST GET server service get all vehicle positions.
     *
     * @return REST response of the type JSON containing all current locations of available vehicles.
     */
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
            Log.logConfig("RACECAR_BACKEND", "All vehicle Locations request has been completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(locations)).
                    type("application/json").
                    build();
        }
        return null;
    }

    /**
     * REST GET server service to get a calculation cost of all available vehicles. It requests from each vehicle a calculation
     * of a possible route and returns a JSON containing all answers.
     *
     * @param startId Starting waypoint ID.
     * @param endId   Ending waypoint ID.
     * @return REST response of the type JSON containg all calculated costs of each vehicle.
     */
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
            int timer = 0;
            for (Vehicle vehicle : vehicles.values()) {
                if (vehicle.isAvailable()) {
                    totalVehicles++;
                    mqttUtils.publishMessage("racecar/" + Long.toString(vehicle.getID()) + "/costrequest", Long.toString(startId) + " " + Long.toString(endId));
                }
            }

            while (costs.size() != totalVehicles && timer != 100) { // Wait for each vehicle to complete the request or timeout after 100 attempts.
                Log.logInfo("RACECAR_BACKEND", "waiting for vehicles to complete request.");
                Thread.sleep(200);
                timer++;
            }
            ArrayList<Cost> costCopy = (ArrayList<Cost>) costs.clone();
            costs.clear();
            Log.logInfo("RACECAR_BACKEND", "Cost calculation request completed.");
            return Response.status(Response.Status.OK).
                    entity(JSONUtils.arrayToJSONString(costCopy)).
                    type("application/json").
                    build();
        }
        return null;
    }

    /**
     * REST GET server service to get the currently used map.
     *
     * @return REST response of the type Text Plain containing the mapname.
     */
    @GET
    @Path("getmapname")
    @Produces("text/plain")
    public String getMapName() {
        return currentMap;
    }

    /**
     * REST GET server service to delete a vehicle. It in returns does a REST GET request as well to the BackBone service
     * to unregister the vehicle from them.
     *
     * @param id ID of the vehicle removing itself.
     */
    @GET
    @Path("delete/{id}")
    @Produces("text/plain")
    public Response deleteVehicle(@PathParam("id") final long id, @Context HttpServletResponse response) throws IOException {
        if (vehicles.containsKey(id)) {
            if (!debugWithoutBackBone)
                restUtilsBackBone.getTextPlain("bot/delete/" + id);
            vehicles.remove(id);
            Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + id + " stopped. ID removed.");
            return Response.status(Response.Status.OK).build();
        } else {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, " vehicle with id " + id + "  not found");
        }
        return null;
    }

    public void deleteVehicle(final long id)
    {
        if (vehicles.containsKey(id)) {
            if (!debugWithoutBackBone)
                restUtilsBackBone.getTextPlain("bot/delete/" + id);
            vehicles.remove(id);
            Log.logInfo("RACECAR_BACKEND", "Vehicle with ID " + id + " stopped. ID removed.");
        } else
            Log.logWarning("RACECAR_BACKEND", "Vehicle with ID " +  id + " could not be deleted as it does not exist");
    }

    /**
     * REST GET server service to get all currently used waypoints by F1 vehicles.
     *
     * @return REST response of the type JSON containing all waypoints.
     */
    @GET
    @Path("getwaypoints")
    @Produces("application/json")
    public String getWayPoints() {
        return JSONUtils.objectToJSONStringWithKeyWord("wayPoints", wayPoints);
    }


    /**
     * REST GET server service to download a map's PGM file by name.
     *
     * @param mapname the name of the map
     * @return REST response of the type Octet-stream containing the file.
     */
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
                System.out.println("error " + e);
                response.sendError(HttpServletResponse.SC_NOT_FOUND, mapname + ".pgm not found");
            }
        };
        return Response
                .ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
                .header("content-disposition", "attachment; filename = " + mapname + ".pgm")
                .build();

    }

    /**
     * REST GET server service to download a map's YAML file by name.
     *
     * @param mapname the name of the map
     * @return REST response of the type Octet-stream containing the file.
     */
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

    /**
     * REST GET server service to make a vehicle do a job. It in return sends a MQTT message to the specified vehicle
     * to execute the job.
     *
     * @param idJob     ID of the job.
     * @param idVehicle ID of the vehicle
     * @param idStart   ID of starting waypoint of the route
     * @param idEnd     ID of starting ending of the route
     */
    @GET
    @Path("executeJob/{idJob}/{idVehicle}/{idStart}/{idEnd}")
    @Produces("text/plain")
    public Response jobRequest(@PathParam("idJob") long idJob, @PathParam("idVehicle") long idVehicle, @PathParam("idStart") long idStart, @PathParam("idEnd") long idEnd, String data, @Context HttpServletResponse response) throws IOException {
        Job job = new Job(idJob, idStart, idEnd, idVehicle);
        if (vehicles.containsKey(job.getIdVehicle())) {
            if (!vehicles.get(job.getIdVehicle()).getOccupied() && vehicles.get(job.getIdVehicle()).isAvailable()) {
                if (wayPoints.containsKey(job.getIdStart())) {
                    if (wayPoints.containsKey(job.getIdEnd())) {
                        vehicles.get(job.getIdVehicle()).setJob(job);
                        Location location = vehicles.get(job.getIdVehicle()).getLocation();
                        location.setIdStart(job.getIdStart());
                        location.setIdEnd(job.getIdEnd());
                        location.setPercentage(0);
                        vehicles.get(job.getIdVehicle()).setLocation(location);
                        jobSend(job.getIdVehicle(), job.getIdStart(), job.getIdEnd());
                        return Response.status(Response.Status.OK).build();
                    } else {
                        response.sendError(HttpServletResponse.SC_NOT_FOUND, "No matching ending waypoint found");
                        Log.logWarning("RACECAR_BACKEND", "Can't send route job as waypoints " + job.getIdStart() + " was not found.");
                    }
                } else {
                    response.sendError(HttpServletResponse.SC_NOT_FOUND, "No matching starting waypoint found");
                    Log.logWarning("RACECAR_BACKEND", "Can't send route job as waypoints " + job.getIdStart() + " was not found.");
                }
            } else {
                response.sendError(HttpServletResponse.SC_FORBIDDEN, "This vehicle is busy or occupied");
                Log.logWarning("RACECAR_BACKEND", "Vehicle with ID " + job.getIdVehicle() + " is occupied or not available. Cant send route job.");
            }

        } else {
            response.sendError(HttpServletResponse.SC_NOT_FOUND, "This vehicle doesn't exist");
            Log.logWarning("RACECAR_BACKEND", "Vehicle with ID " + job.getIdVehicle() + " doesn't exist. Cant send route job.");
        }
        return Response.status(Response.Status.OK).build();
    }

    /**
     * Rest command that can be called to change the map used by the racecars at runtime
     * @param mapName name of the new map
     * @return
     */
    @GET
    @Path("changeMap/{mapName}")
    @Produces("text/plain")
    public String changeMap(@PathParam("mapName") String mapName)
    {
        File f = new File(mapsPath + "/" + mapName + ".yaml");
        if(f.exists() && !f.isDirectory()) {
            currentMap=mapName;
            for(long ID : vehicles.keySet()) {
                log.logInfo("RACECAR_BACKEND", "change map command send to vehicle with ID: " + ID);
                mqttUtils.publishMessage("racecar/" + ID + "/changeMap", mapName);
                loadWayPoints();
            }
            return "Command was executed to change map";
        }
        else {
            log.logWarning("RACECAR_BACKEND", "Map cannot be changed as the map does not exist");
            return "Map was not changed as map does not exist";
        }

    }

    /**
     * Method to send a MQTT message to a specified vehicle to execute a job.
     *
     * @param ID      ID of the vehicle
     * @param startID ID of starting waypoint of the route
     * @param endID   ID of endID waypoint of the route
     */
    private void jobSend(long ID, long startID, long endID) {
        vehicles.get(ID).setOccupied(true);
        Log.logInfo("RACECAR_BACKEND", "Route job send to vehicle with ID " + ID + " from " + startID + " to " + endID + ".");
        mqttUtils.publishMessage("racecar/" + ID + "/job", Long.toString(startID) + " " + Long.toString(endID));
    }

    public static HashMap<Long, Vehicle> getVehicles() {
        return vehicles;
    }

    public static void main(String[] args) throws Exception {
            final RacecarBackend racecarBackend = new RacecarBackend(true);

    }
}

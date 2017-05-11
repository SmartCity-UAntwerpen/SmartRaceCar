package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import org.eclipse.paho.client.mqttv3.MqttException;

import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.*;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashMap;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener {

    private static Log log;
    private Level level = Level.INFO; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "root";
    private final String mqttPassword = "smartcity";
    private final String mapFolder = "maps";
    private final String wayPointFolder = "waypoints";

    private MQTTUtils mqttUtils;
    private JSONUtils jsonUtils;

    private static HashMap<Integer, WayPoint> waypoints = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<Long, Vehicle> vehicles = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    private static HashMap<String, Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private static String currentMap;

    public Manager() {

    }

    public Manager(String currentMap) throws MqttException {
        log = new Log(this.getClass(), level);
        Manager.currentMap = currentMap;
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(mqttBroker, mqqtUsername, mqttPassword, this);
        mqttUtils.subscribeToTopic("racecar/#");
        loadedMaps = XMLUtils.loadMaps(mapFolder);
        waypoints = XMLUtils.loadWaypoints(wayPointFolder);
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
                if(!vehicles.get(ID).getOccupied()){
                    int n = waypointValues.length - 1;
                    long[] newArray = new long[n];
                    System.arraycopy(waypointValues, 1, newArray, 0, n);
                    jobSend(ID, newArray);
                }else{
                    Log.logWarning("MANAGER", "Vehicle with ID " + ID + " is occupied. Cant send route job.");
                }

            } else {
                Log.logWarning("MANAGER", "Vehicle with ID " + ID + " doesn't exist. Cant send route job.");
            }
        } else if (topic.matches("racecar/[0-9]+/location")) {
            if (vehicles.containsKey(Long.parseLong(topic.replaceAll("\\D+", "")))) {
                long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
                String[] locationValues = message.split(" ");
                Float x = Float.parseFloat(locationValues[0]);
                Float y = Float.parseFloat(locationValues[1]);
                Float z = Float.parseFloat(locationValues[2]);
                Float w = Float.parseFloat(locationValues[3]);
                vehicles.get(ID).setPoint(new Point(x, y, z, w));
                log.logInfo("MANAGER", "Location update of vehicle with ID " + ID + ". New location:" + x + "," + y + "," + z + "," + w);
            } else {
                Log.logWarning("MANAGER", "Vehicle with ID " + Long.parseLong(topic.replaceAll("\\D+", "")) + " doesn't exist. Cant set location.");
            }
        } else if (topic.matches("racecar/[0-9]+/routecomplete")) {
            if (message.equals("done")) {
                long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
                vehicles.get(ID).setOccupied(false);
                log.logInfo("MANAGER", "Vehicle with ID " + ID + " has completed his route.");
            } else if (message.equals("error")) {
                long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
                vehicles.get(ID).setOccupied(false);
                log.logInfo("MANAGER", "Vehicle with ID " + ID + " had errors in his route request.");
            } else if (message.equals("notcomplete")) {
                long ID = Long.parseLong(topic.replaceAll("\\D+", ""));
                vehicles.get(ID).setOccupied(true);
                log.logInfo("MANAGER", "Vehicle with ID " + ID + " didn't complete his route yet.");
            }
        }
    }

    private void jobSend(long ID, long[] waypointValues) {
        vehicles.get(ID).setOccupied(true);
        String message = Arrays.toString(waypointValues).replace(", ", " ").replace("[", "").replace("]", "").trim();
        log.logInfo("MANAGER", "Route job send to vehicle with ID " + ID + " with waypoints " + message);
        mqttUtils.publishMessage("racecar/" + ID + "/job", message);
    }

    @GET
    @Path("register")
    @Produces("text/plain")
    public long register(@DefaultValue("0") @QueryParam("x") float x, @DefaultValue("0") @QueryParam("y") float y, @DefaultValue("0") @QueryParam("z") float z, @DefaultValue("0") @QueryParam("w") float w) {
        long id = vehicles.size();
        vehicles.put(id, new Vehicle(vehicles.size(), false, new Point(x, y, z, w)));
        log.logInfo("MANAGER", "New vehicle registered. Given ID " + (vehicles.size() - 1) + ". Has start position: " + x + "," + y + "," + z + "," + w);
        return id;
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
        return JSONUtils.objectToJSONString("waypoints", waypoints);
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
                response.sendError(response.SC_NOT_FOUND, mapname + ".pgm not found");
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
                response.sendError(response.SC_NOT_FOUND, mapname + ".yaml not found");
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
}

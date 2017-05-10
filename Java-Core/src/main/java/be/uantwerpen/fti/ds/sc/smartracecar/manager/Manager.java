package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener{

    private static Log log;
    private Level level = Level.CONFIG; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "username";
    private final String mqttPassword = "password";
    private final String mapFolder = "maps";
    private final String wayPointFolder = "waypoints";

    private MQTTUtils mqttUtils;
    private JSONUtils jsonUtils;

    static HashMap<Integer,WayPoint> waypoints = new HashMap<>(); // ArrayList of all vehicles mapped by ID.
    static ArrayList<Vehicle> vehicles = new ArrayList<>(); // ArrayList of all vehicles mapped by ID.
    static HashMap<String,Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    static String currentMap;

    public Manager(){

    }

    public Manager(String currentmap){
        log = new Log(this.getClass(),level);
        this.currentMap = currentmap;
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(mqttBroker,mqqtUsername,mqttPassword,this);
        mqttUtils.subscribeToTopic("racecar/#");
        loadedMaps = XMLUtils.loadMaps(mapFolder);
        waypoints = XMLUtils.loadWaypoints(wayPointFolder);
    }

    @Override
    public void parseMQTT(String topic, String message) {

    }

    @GET
    @Path("register")
    @Produces("text/plain")
    public int registerREST(@DefaultValue("0") @QueryParam("x") float x, @DefaultValue("0") @QueryParam("y") float y, @DefaultValue("0") @QueryParam("z") float z, @DefaultValue("0") @QueryParam("w") float w) {
        vehicles.add(new Vehicle(vehicles.size(),false,new Point(x,y,z,w)));
        return vehicles.size()-1;
    }

    @GET
    @Path("getmapname")
    @Produces("text/plain")
    public String getMapName(){
        return currentMap;
    }

    @GET
    @Path("getwaypoints")
    @Produces("application/json")
    public String getWayPoints(){
        return jsonUtils.objectToJSONString("waypoints",waypoints);
    }


    @GET
    @Path("getmappgm/{mapname}")
    @Produces("application/octet-stream")
    public Response getMapPGM(@PathParam("mapname") final String mapname){

        StreamingOutput fileStream =  new StreamingOutput()
        {
            @Override
            public void write(java.io.OutputStream output) throws IOException, WebApplicationException
            {
                try
                {
                    java.nio.file.Path path = Paths.get("maps/" + mapname + ".pgm");
                    byte[] data = Files.readAllBytes(path);
                    output.write(data);
                    output.flush();
                }
                catch (Exception e)
                {
                    throw new WebApplicationException("File Not Found !!");
                }
            }
        };
        return Response
                .ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
                .header("content-disposition","attachment; filename = " + mapname + ".pgm")
                .build();
    }

    @GET
    @Path("getmapyaml/{mapname}")
    @Produces("application/octet-stream")
    public Response getMapYAML(@PathParam("mapname") final String mapname){

        StreamingOutput fileStream =  new StreamingOutput()
        {
            @Override
            public void write(java.io.OutputStream output) throws IOException, WebApplicationException
            {
                try
                {
                    java.nio.file.Path path = Paths.get("maps/" + mapname + ".yaml");
                    byte[] data = Files.readAllBytes(path);
                    output.write(data);
                    output.flush();
                }
                catch (Exception e)
                {
                    throw new WebApplicationException("File Not Found !!");
                }
            }
        };
        return Response
                .ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
                .header("content-disposition","attachment; filename = " + mapname + ".yaml")
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

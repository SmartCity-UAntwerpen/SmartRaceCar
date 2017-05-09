package be.uantwerpen.fti.ds.smartracecar.manager;

import be.uantwerpen.fti.ds.smartracecar.common.*;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.logging.Level;

@Path("carmanager")
public class Manager implements MQTTListener{

    private Log log;
    private Level level = Level.INFO; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "username";
    private final String mqttPassword = "password";
    private final String mapFolder = "maps";

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private JSONUtils jsonUtils;

    private HashMap<Integer,Vehicle> vehicles; // ArrayList of all vehicles mapped by ID.
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private static String currentMap;
    private int counter = 0;

    public Manager() throws InterruptedException {
        vehicles = new HashMap<>();
        log = new Log(this.getClass(),level);
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(mqttBroker,mqqtUsername,mqttPassword,this);
        mqttUtils.subscribeToTopic("racecar/#");
        loadedMaps = XMLUtils.loadMaps(mapFolder);
    }

    @Override
    public void parseMQTT(String topic, String message) {

    }

    @GET
    @Path("register")
    @Produces("text/plain")
    public int registerREST(@DefaultValue("0") @QueryParam("x") float x,@DefaultValue("0") @QueryParam("y") float y,@DefaultValue("0") @QueryParam("z") float z,@DefaultValue("0") @QueryParam("w") float w) {
        counter++;
        log.logInfo("MANAGER",x + ","+ y + "," + z + "," + w + "=" + counter);
        return counter;
    }

    @GET
    @Path("getmapname")
    @Produces("text/plain")
    public String getMapName(){
        return currentMap;
    }

    @GET
    @Path("getmap/{mapname}")
    @Produces(MediaType.APPLICATION_OCTET_STREAM)
    public Response getMap(@PathParam("mapname") final String mapname){

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

    public static void main(String[] args) throws Exception {
        if (args.length == 0) {
            System.out.println("Need at least 1 argument to run. Possible arguments: currentMap(String)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) currentMap = args[0];
        }
        final Manager manager = new Manager();
        new TomCatLauncher().start();


    }
}

package be.uantwerpen.fti.ds.smartracecar.manager;

import be.uantwerpen.fti.ds.smartracecar.common.*;
import java.util.HashMap;
import java.util.logging.Level;

public class Manager implements MQTTListener{

    private Log log;
    private Level level = Level.CONFIG; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "username";
    private final String mqttPassword = "password";
    private final String mapFolder = "maps";

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private JSONUtils jsonUtils;

    private HashMap<Integer,Vehicle> vehicles = new HashMap<>(); // Map of all vehicles mapped by ID.
    private HashMap<String,Map> loadedMaps = new HashMap<>(); // Map of all loaded maps.
    private static String currentMap;

    public Manager() throws InterruptedException {
        log = new Log(this.getClass(),level);
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(mqttBroker,mqqtUsername,mqttPassword,this);
        mqttUtils.subscribeToTopic("racecar/#");
        loadedMaps = XMLUtils.loadMaps(mapFolder);
        while(true){

            log.logInfo("MANAGER","test");
            Thread.sleep(2000);
        }
    }

    @Override
    public void parseMQTT(String topic, String message) {

    }

    public static void main(String[] args) throws InterruptedException {
        if (args.length == 0) {
            System.out.println("Need at least 1 argument to run. Possible arguments: currentMap(String)");
            System.exit(0);
        } else if (args.length == 1) {
            if (!args[0].isEmpty()) currentMap = args[0];
        }
        final Manager manager = new Manager();
    }
}

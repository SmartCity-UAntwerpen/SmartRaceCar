package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import java.io.IOException;
import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.logging.Level;

 class SimDeployer implements TCPListener {

    private final int serverPort = 9999;
    private static Log log;
    private Level level = Level.CONFIG;
    private final String restURL = "http://143.129.39.151:8081/carmanager"; // REST Service URL to Manager


     private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    private static HashMap<Long, SimulatedVehicle> simulatedVehicles = new HashMap<>();private HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.

    private SimDeployer() throws IOException {
        log = new Log(this.getClass(), level);
        restUtils = new RESTUtils(restURL);
        requestWaypoints();
        tcpUtils = new TCPUtils(serverPort, this,true);
        tcpUtils.start();
    }

     //Request all possible waypoints from RaceCarManager
     private void requestWaypoints() {
         Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>() {}.getType();
         wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
         assert wayPoints != null;
         for (WayPoint wayPoint : wayPoints.values()) {
             Log.logConfig("SIMDEPLOYER", "Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
         }
         Log.logInfo("SIMDEPLOYER", "All possible waypoints(" + wayPoints.size() + ") received.");
     }

    @Override
    public String parseTCP(String message) throws IOException {
        boolean result = false;
        if (message.matches("create\\s[0-9]+")) {
            result = createVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("run\\s[0-9]+")) {
            result = startupVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("stop\\s[0-9]+")) {
            result = stopVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("kill\\s[0-9]+")) {
            result = killVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("restart\\s[0-9]+")) {
            result = restartVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("set\\s[0-9]+\\s\\w+\\s\\w+")) {
            String[] splitString = message.split("\\s+");
            Long simulationID = Long.parseLong(splitString[1]);
            String parameter = splitString[2];
            String argument = splitString[3];
            result = setVehicle(simulationID,parameter,argument);
        }
        if(result){
            return "ACK";
        }
        else{
            return "NACK";
        }
    }

    private boolean setVehicle(long simulationID, String parameter, String argument) {
        if (simulatedVehicles.containsKey(simulationID)) {
            switch (parameter) {
                case "startpoint":
                    if(wayPoints.containsKey(Long.parseLong(argument))) {
                        simulatedVehicles.get(simulationID).setStartPoint(Long.parseLong(argument));
                        Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " given starting point ID " + argument + ".");
                        return true;
                    }else{
                        Log.logWarning("SIMDEPLOYER", "Cannot set vehicle with simulation ID " + simulationID + " to have startpoint " +  Long.parseLong(argument) + ". It does not exist.");
                        return false;
                    }
                case "speed":
                    simulatedVehicles.get(simulationID).setSpeed(Float.parseFloat(argument));
                    Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " given speed " + argument + ".");
                    return true;
                case "name":
                    simulatedVehicles.get(simulationID).setName(argument);
                    Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " given name " + argument + ".");
                    return true;
                default:
                    Log.logWarning("SIMDEPLOYER", "No matching keyword when parsing simulation request over sockets. Parameter: " + parameter);
                    return false;
            }
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot set vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean createVehicle(long simulationID){
        if (!simulatedVehicles.containsKey(simulationID)) {
            simulatedVehicles.put(simulationID, new SimulatedVehicle(simulationID));
            //TODO deploy jars
            Log.logInfo("SIMDEPLOYER", "New simulated vehicle registered with simulation ID " + simulationID + ".");
            return true;
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot create vehicle with simulation ID " + simulationID + ". It already exists.");
            return false;
        }
    }

    private boolean stopVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            Log.logInfo("SIMDEPLOYER", "Vehicle with ID " + simulationID + " Stopped.");
            return true;
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean killVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            simulatedVehicles.remove(simulationID);
            Log.logInfo("SIMDEPLOYER", "Vehicle with ID " + simulationID + " killed.");
            return true;
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean restartVehicle(long simulationID){
        if (simulatedVehicles.containsKey(simulationID)) {
            if (simulatedVehicles.get(simulationID).isDeployed()) {
                Log.logInfo("SIMDEPLOYER", "Restarted vehicle with simulation ID " + simulationID + ".");
                return true;
            } else {
                Log.logWarning("SIMDEPLOYER", "Cannot restart vehicle with simulation ID " + simulationID + ". It isn't started.");
                return false;
            }
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean startupVehicle(long simulationID){
        if (simulatedVehicles.containsKey(simulationID)) {
            if (!simulatedVehicles.get(simulationID).isDeployed()) {
                if (simulatedVehicles.get(simulationID).getStartPoint() != -1) {
                    simulatedVehicles.get(simulationID).setDeployed(true);
                    Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " started.");
                    return true;
                } else {
                    Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It didn't have a starting point set.");
                    return false;
                }
            } else {
                Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It was already started.");
                return false;
            }
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }


    public static void main(String[] args) throws IOException {
        final SimDeployer simDeployer = new SimDeployer();
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;

import java.io.IOException;
import java.util.HashMap;
import java.util.logging.Level;

 class SimDeployer implements TCPListener {

    private final int serverPort = 5007;
    private static Log log;
    private Level level = Level.CONFIG;

    private TCPUtils tcpUtils;

    private static HashMap<Long, SimulatedVehicle> simulatedVehicles = new HashMap<>();

    private SimDeployer() throws IOException {
        log = new Log(this.getClass(), level);
        tcpUtils = new TCPUtils(serverPort, this);
        tcpUtils.start();
    }

    @Override
    public String parseTCP(String message) {
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
        if (!simulatedVehicles.containsKey(simulationID)) {
            switch (parameter) {
                case "startpoint":
                    simulatedVehicles.get(simulationID).setStartPoint(Long.parseLong(argument));
                    Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID + " given starting point ID " + argument + ".");
                    return true;
                case "speed":
                    simulatedVehicles.get(simulationID).setSpeed(Float.parseFloat(argument));
                    Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID + " given speed " + argument + ".");
                    return true;
                case "name":
                    simulatedVehicles.get(simulationID).setName(argument);
                    Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID + " given name " + argument + ".");
                    return true;
                default:
                    Log.logWarning("CORE", "No matching keyword when parsing simulation request over sockets. Parameter: " + parameter);
                    return false;
            }
        } else {
            Log.logWarning("MANAGER", "Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean createVehicle(long simulationID){
        if (!simulatedVehicles.containsKey(simulationID)) {
            simulatedVehicles.put(simulationID, new SimulatedVehicle(simulationID));
            Log.logInfo("MANAGER", "New simulated vehicle registered with simulation ID " + simulationID + ".");
            return true;
        } else {
            Log.logWarning("MANAGER", "Cannot create vehicle with simulation ID " + simulationID + ". It already exists.");
            return false;
        }

    }

    private boolean stopVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            Log.logInfo("MANAGER", "Vehicle with ID " + simulationID + " Stopped.");
            return true;
        } else {
            Log.logWarning("MANAGER", "Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean killVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            simulatedVehicles.remove(simulationID);
            Log.logInfo("MANAGER", "Vehicle with ID " + simulationID + " killed.");
            return true;
        } else {
            Log.logWarning("MANAGER", "Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean restartVehicle(long simulationID){
        if (simulatedVehicles.containsKey(simulationID)) {
            if (simulatedVehicles.get(simulationID).isDeployed()) {
                Log.logInfo("MANAGER", "Restarted vehicle with simulation ID " + simulationID + ".");
                return true;
            } else {
                Log.logWarning("MANAGER", "Cannot restart vehicle with simulation ID " + simulationID + ". It isn't started.");
                return false;
            }
        } else {
            Log.logWarning("MANAGER", "Cannot restart vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    private boolean startupVehicle(long simulationID){
        if (simulatedVehicles.containsKey(simulationID)) {
            if (!simulatedVehicles.get(simulationID).isDeployed()) {
                if (simulatedVehicles.get(simulationID).getStartPoint() == -1) {
                    simulatedVehicles.get(simulationID).setDeployed(true);
                    Log.logInfo("MANAGER", "Simulated Vehicle with simulation ID " + simulationID + " started.");
                    return true;
                } else {
                    Log.logWarning("MANAGER", "Cannot start vehicle with simulation ID " + simulationID + ". It didn't have a starting point set.");
                    return false;
                }
            } else {
                Log.logWarning("MANAGER", "Cannot start vehicle with simulation ID " + simulationID + ". It was already started.");
                return false;
            }
        } else {
            Log.logWarning("MANAGER", "Cannot start vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }


    public static void main(String[] args) throws Exception {
        final SimDeployer simDeployer = new SimDeployer();
    }
}

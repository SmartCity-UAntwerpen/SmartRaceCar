package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Type;
import java.net.URLDecoder;
import java.util.HashMap;
import java.util.Properties;
import java.util.logging.Level;

/**
 * Module that handles all simulated vehicles and communicates with the SimWorker service to setup new
 * simulated F1 cars. It deploys and manages the '.jar' versions of the vehicle simulation.
 */
class SimDeployer implements TCPListener {

    //Standard settings (without config file loaded)
    private int serverPort = 9999; // Port to communicate to SimWorker over.
    private String restURL = "http://smartcity.ddns.net:8081/carmanager"; // REST Service URL to Manager

    //Help services
    private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    private Log log; // logging instance
    private String jarPath = ".\\release\\";//"C:\\release\\";  //Path where the jar files are located.
    private HashMap<Long, SimulatedVehicle> simulatedVehicles = new HashMap<>(); // Map of all simulated vehicles. Mapped by their ID.
    private HashMap<Long, WayPoint> wayPoints = new HashMap<>(); // Map of all loaded waypoints.

    /**
     * Module that handles all simulated vehicles and communicates with the SimWorker service to setup new
     * simulated F1 cars. It deploys and manages the '.jar' versions of the vehicle simulation.
     */
    SimDeployer() throws IOException, InterruptedException {
        String asciiArt1 = FigletFont.convertOneLine("SmartCity");
        System.out.println(asciiArt1);
        System.out.println("-------------------------------------------------------------------");
        System.out.println("----------------- F1 Racecar SimDeployer - v1.0 -------------------");
        System.out.println("-------------------------------------------------------------------");
        loadConfig();
        this.jarPath = jarPath;
        System.out.println(jarPath);
        restUtils = new RESTUtils(restURL);
        requestWaypoints();
        tcpUtils = new TCPUtils(serverPort, this);
        tcpUtils.start();
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
            String path = SimDeployer.class.getProtectionDomain().getCodeSource().getLocation().getPath();
            String decodedPath = URLDecoder.decode(path, "UTF-8");
            decodedPath = decodedPath.replace("SimDeployer.jar", "");
            input = new FileInputStream(decodedPath + "/simdeployer.properties");
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
            restURL = prop.getProperty("restURL");
            jarPath = prop.getProperty("jarPath");
            serverPort = Integer.parseInt(prop.getProperty("serverPort"));
            Log.logInfo("SIMDEPLOYER", "Config loaded");
        } catch (IOException ex) {
            log = new Log(this.getClass(), Level.CONFIG);
            Log.logWarning("SIMDEPLOYER", "Could not read config file. Loading default settings. " + ex);
        } finally {
            if (input != null) {
                try {
                    input.close();
                } catch (IOException e) {
                    Log.logWarning("SIMDEPLOYER", "Could not read config file. Loading default settings. " + e);
                }
            }
        }
    }


    /**
     * Request all possible waypoints from the Manager through a REST GET request.
     */
    private void requestWaypoints() {
        Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>() {
        }.getType();
        wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
        assert wayPoints != null;
        for (WayPoint wayPoint : wayPoints.values()) {
            Log.logConfig("SIMDEPLOYER", "Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
        }
        Log.logInfo("SIMDEPLOYER", "All possible waypoints(" + wayPoints.size() + ") received.");
    }


    /**
     * Interfaced method to parse TCP message socket callback is triggered by incoming message. Used to
     * create, run , set, stop , kill or restart simulated vehicles by handling these incoming request from the SimWorker.
     *
     * @param message received TCP socket message string
     * @return a return answer to be send back over the socket to the SimWorker. Here being ACK or NACK.
     */
    @Override
    public String parseTCP(String message) throws IOException {
        boolean result = false;
        if (message.matches("create\\s[0-9]+")) {
            result = createVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
        } else if (message.matches("run\\s[0-9]+")) {
            result = runVehicle(Long.parseLong(message.replaceAll("\\D+", "")));
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
            result = setVehicle(simulationID, parameter, argument);
        }
        if (result) {
            return "ACK";
        } else {
            return "NACK";
        }
    }

    /**
     * Called when a simulated vehicle needs it's settings changed. Can be it's name, startpoint or speed.
     *
     * @param simulationID The ID of the vehicle that needs to be set.
     * @param parameter    The specific property parameter (name,startpoint or speed)
     * @param argument     the argument value of this property.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean setVehicle(long simulationID, String parameter, String argument) {
        if (simulatedVehicles.containsKey(simulationID)) {
            switch (parameter) {
                case "startpoint":
                    if (wayPoints.containsKey(Long.parseLong(argument))) {
                        simulatedVehicles.get(simulationID).setStartPoint(Long.parseLong(argument));
                        if (simulatedVehicles.get(simulationID).isDeployed()) {
                            simulatedVehicles.get(simulationID).setStartPoint(Long.parseLong(argument));
                            if (!simulatedVehicles.get(simulationID).isAvailable())
                                simulatedVehicles.get(simulationID).setStartPoint();
                        }
                        Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " given starting point ID " + argument + ".");
                        return true;
                    } else {
                        Log.logWarning("SIMDEPLOYER", "Cannot set vehicle with simulation ID " + simulationID + " to have startpoint " + Long.parseLong(argument) + ". It does not exist.");
                        return false;
                    }
                case "speed":
                    try{
                        simulatedVehicles.get(simulationID).setSpeed(Float.parseFloat(argument));
                    }catch(NumberFormatException ex) {
                        Log.logWarning("SIMDEPLOYER", "Needs to be a float number as speed argument. Argument: " + argument);
                        return false;
                    }

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

    /**
     * Called when a simulated vehicle needs to be created. This doesn't start the simulation yet but makes it ready
     * in the backend systems and in the SimDeployer, ready to be altered and managed.
     *
     * @param simulationID The ID of the vehicle that needs to be created.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean createVehicle(long simulationID) {
        if (!simulatedVehicles.containsKey(simulationID)) {
            simulatedVehicles.put(simulationID, new SimulatedVehicle(simulationID, jarPath));
            Log.logInfo("SIMDEPLOYER", "New simulated vehicle registered with simulation ID " + simulationID + ".");
            return true;
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot create vehicle with simulation ID " + simulationID + ". It already exists.");
            return false;
        }
    }

    /**
     * Called when a simulated vehicle needs to be stopped. This doesn't remove the vehicle from the systems or ends
     * the simulation. It simply disables the vehicle from receiving any jobs or requests and makes it hidden from the
     * visualization. This allows it to be altered or restarted.
     *
     * @param simulationID The ID of the vehicle that needs to be stopped.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean stopVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            if(simulatedVehicles.get(simulationID).isDeployed()){
                simulatedVehicles.get(simulationID).stop();
                Log.logInfo("SIMDEPLOYER", "Vehicle with ID " + simulationID + " Stopped.");
                return true;
            }else{
                Log.logWarning("SIMDEPLOYER", "Cannot stop vehicle with simulation ID " + simulationID + ". It isn't deployed yet.");
                return false;
            }
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot stop vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    /**
     * Called when a simulated vehicle needs to be removed. This stops the simulation no matter what state it is in.
     * It does request a unregistering from the backBone however through the simulated Core.
     *
     * @param simulationID The ID of the vehicle that needs to be killed.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean killVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            if (simulatedVehicles.get(simulationID).isDeployed()) simulatedVehicles.get(simulationID).kill();
            simulatedVehicles.remove(simulationID);
            Log.logInfo("SIMDEPLOYER", "Vehicle with ID " + simulationID + " killed.");
            return true;
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot kill vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }

    /**
     * Called when a simulated vehicle needs to be restarted. This makes it restart at it's currently set startpoint
     * and if the vehicle was stopped makes it available again for jobs and requests.
     *
     * @param simulationID The ID of the vehicle that needs to be restarted.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean restartVehicle(long simulationID) {
        if (simulatedVehicles.containsKey(simulationID)) {
            if (simulatedVehicles.get(simulationID).isDeployed()) {
                Log.logInfo("SIMDEPLOYER", "Restarted vehicle with simulation ID " + simulationID + ".");
                simulatedVehicles.get(simulationID).restart();
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

    /**
     * Called when a simulated vehicle needs to be run. This makes it start the simulation(if it wasn't started already).
     * If the vehicle was stopped then it simply unpauses it and makes it available again for jobs and requests.
     * IF the vehicle wasn't run yet this will make the simulation actually start by launching the jars.
     *
     * @param simulationID The ID of the vehicle that needs to be restarted.
     * @return A boolean to signify if the request could be handled or not. False means it could not be processed.
     */
    private boolean runVehicle(long simulationID) throws IOException {
        if (simulatedVehicles.containsKey(simulationID)) {
            if (!simulatedVehicles.get(simulationID).isDeployed()) {
                if (simulatedVehicles.get(simulationID).getStartPoint() != -1) {
                    simulatedVehicles.get(simulationID).start(tcpUtils.findRandomOpenPort(), tcpUtils.findRandomOpenPort());
                    Log.logInfo("SIMDEPLOYER", "Simulated Vehicle with simulation ID " + simulationID + " started.");
                    return true;
                } else {
                    Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It didn't have a starting point set.");
                    return false;
                }
            } else {
                if (!simulatedVehicles.get(simulationID).isAvailable()) {
                    simulatedVehicles.get(simulationID).run();
                    Log.logInfo("SIMDEPLOYER", "Stopped simulated Vehicle with simulation ID " + simulationID + " started again.");
                    return true;
                } else {
                    Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It was already started.");
                    return false;
                }
            }
        } else {
            Log.logWarning("SIMDEPLOYER", "Cannot start vehicle with simulation ID " + simulationID + ". It does not exist.");
            return false;
        }
    }


    public static void main(String[] args) throws IOException, InterruptedException {
            SimDeployer simDeployer = new SimDeployer();
    }
}

package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.*;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Type;
import java.net.URISyntaxException;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Properties;
import java.util.logging.Level;

/**
 * Module representing the high-level of a vehicle.
 */
class Core implements TCPListener, MQTTListener
{
	private Log oldlog;

	private static long startPoint;                                        // Starting position on map. Given by main argument.

	//Standard settings (without config file loaded)
	private boolean debugWithoutRosKernel = false;                            // debug parameter for using this module without a connected RosKernel/SimKernel

	private boolean occupied;                                                // To verify if racecar is currently occupied by a route job.

	//Help services
	private MQTTUtils mqttUtils;
	private TCPUtils tcpUtils;
	private RESTUtils restUtils;
	private HeartbeatPublisher heartbeatPublisher;

	//variables
	private long ID;                                                        // ID given by RacecarBackend to identify vehicle.
	private LogbackWrapper log;                                                        // logging instance
	private HashMap<Long, WayPoint> wayPoints;                                // Map of all loaded waypoints.
	private boolean connected = false;                                        // To verify socket connection to vehicle.

	private Navigator navigator;
	private WeightManager weightManager;
	private CoreParameters params;
	private MapManager mapManager;


	/**
	 * Module representing the high-level of a vehicle.
	 *
	 * @param startPoint Starting point of the vehicle. Defined by input arguments of main method.
	 * @param serverPort Port to listen for messages of SimKernel/Roskernel. Defined by input arguments of main method.
	 * @param clientPort Port to send messages to SimKernel/Roskernel. Defined by input arguments of main method.
	 */
	public Core(long startPoint, int serverPort, int clientPort, CoreParameters params) throws InterruptedException, IOException
	{
		String asciiArt1 = FigletFont.convertOneLine("SmartCity");
		System.out.println(asciiArt1);
		System.out.println("------------------------------------------------------------------");
		System.out.println("--------------------- F1 Racecar Core - v1.0 ---------------------");
		System.out.println("------------------------------------------------------------------");

		this.params = params;

		this.log = new LogbackWrapper(Core.class);

		Core.startPoint = startPoint;

		this.wayPoints = new HashMap<>();
		this.occupied = false;

		try
		{
			this.loadConfig();
		} catch (URISyntaxException e)
		{
			e.printStackTrace();
		}
		this.log.info("CORE", "Startup parameters: Starting Waypoint:" + startPoint + " | TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
		this.restUtils = new RESTUtils(this.params.getRESTCarmanagerURL());
		this.requestWaypoints();
		this.register();
        /*Runtime.getRuntime().addShutdownHook(
                new Thread(new Runnable() {public void run() {
                    Thread thread = new Thread(new Runnable() {public void run(){
                        killCar();
                    }});
                    thread.start();
                    long endTimeMillis = System.currentTimeMillis() + 10000; //10 second timeout
                    while (thread.isAlive()) {
                        if (System.currentTimeMillis() > endTimeMillis) {
                            this.log.warning("CORE", "Timeout was exceeded on exiting the system");
                            System.exit(1);
                        }
                        try {
                            Thread.sleep(500);
                        }
                        catch (InterruptedException t) {}
                    }
            }
        }));*/

		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic("racecar/" + ID + "/#");

		this.tcpUtils = new TCPUtils(clientPort, serverPort, this);
		this.tcpUtils.start();

		if (!this.debugWithoutRosKernel)
		{
			connectSend();
		}

		this.mapManager = new MapManager(this);
		this.mapManager.requestMap();
		this.log.info("CORE", "Giving the map 10s to load.");
		Thread.sleep(10000); //10 seconds delay so the map can load before publishing the startpoint
		sendStartPoint();
		this.log.info("CORE", "Startpoint was send");
		this.heartbeatPublisher = new HeartbeatPublisher(new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this), this.ID);
		this.heartbeatPublisher.start();
		this.log.info("CORE", "Heartbeat publisher was started.");

		this.navigator = new Navigator(this);
		this.log.info("CORE", "Navigator was started.");

		this.weightManager = new WeightManager(this);
		this.log.info("CORE", "Weightmanager was started");
	}

	public HashMap<Long, WayPoint> getWayPoints()
	{
		return this.wayPoints;
	}

	public long getID()
	{
		return this.ID;
	}

	public CoreParameters getParams()
	{
		return this.params;
	}

	public TCPUtils getTcpUtils()
	{
		return this.tcpUtils;
	}

	public MQTTUtils getMqttUtils()
	{
		return this.mqttUtils;
	}

	public boolean isOccupied()
	{
		return this.occupied;
	}

	public void setOccupied(boolean occupied)
	{
		this.occupied = occupied;
	}

	public RESTUtils getRestUtils()
	{
		return this.restUtils;
	}

	/**
	 * Called by incoming timing calculation requests. Sends the request further to the RosKernel/SimKernel.
	 *
	 * @param wayPointIDs Array of waypoint ID's to have their timing calculated.
	 */
	public void timeRequest(long[] wayPointIDs)
	{
		List<Point> points = new ArrayList<>();
		points.add(this.wayPoints.get(wayPointIDs[0]));
		points.add(this.wayPoints.get(wayPointIDs[1]));
		if (!this.params.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("costtiming", points));
		} else
		{
			this.weightManager.costComplete(new Cost(false, 5, 5, this.ID));
		}
	}

	/**
	 * Help method to load all configuration parameters from the properties file with the same name as the class.
	 * If it's not found then it will use the default ones.
	 */
	private void loadConfig() throws URISyntaxException
	{
		/*Properties prop = new Properties();
		InputStream input = null;
		try
		{
			//String path = Core.class.getProtectionDomain().getCodeSource().getLocation().getPath();
			String path =  Core.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();
			this.log.info("CORE", "loading parameters from: " + path);
			String decodedPath = URLDecoder.decode(path, "UTF-8");
			decodedPath = decodedPath.replace("Core.jar", "");
			input = new FileInputStream(decodedPath + "/src/main/core.properties");
			prop.load(input);
			String debugLevel = prop.getProperty("debugLevel");

			switch (debugLevel)
			{
				case "debug":
					oldlog = new Log(this.getClass(), Level.CONFIG);
					break;
				case "info":
					oldlog = new Log(this.getClass(), Level.INFO);
					break;
				case "warning":
					oldlog = new Log(this.getClass(), Level.WARNING);
					break;
				case "severe":
					oldlog = new Log(this.getClass(), Level.SEVERE);
					break;
			}
			this.params.setDebug(Boolean.parseBoolean(prop.getProperty("debugWithoutRosKernel")));
			this.params.setMqttBroker("tcp://" + prop.getProperty("mqttBroker"));
			this.params.setMqttUserName(prop.getProperty("mqqtUsername"));
			this.params.setMqttPassword(prop.getProperty("mqttPassword"));
			this.params.setRestCarmanagerURL(prop.getProperty("restURL"));
			this.log.info("CORE", "Config loaded");
		} catch (IOException ex)
		{
			oldlog = new Log(this.getClass(), Level.SEVERE);
			this.log.warning("CORE", "Could not read config file. Loading default settings. " + ex);

		} finally
		{
			if (input != null)
			{
				try
				{
					input.close();
				} catch (IOException e)
				{
					this.log.warning("CORE", "Could not read config file. Loading default settings. " + e);
				}
			}
		}*/

		CoreParameterParser parser = new CoreParameterParser();
		this.params = parser.parse("/home/ubuntu/Git/SmartRacecar/SmartRaceCar/release/core.properties");
	}


	/**
	 * Send connection request over sockets to RosKernel/SimKernel.
	 */
	private void connectSend()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
	}

	/**
	 * Event to be called when connection to car has been made.
	 */
	private void connectReceive()
	{
		this.log.info("CORE", "Connected to car.");
	}

	/**
	 * Register vehicle with RacecarBackend over REST.
	 */
	private void register()
	{
		String id = this.restUtils.getTextPlain("register/" + Long.toString(this.startPoint));
		this.ID = Long.parseLong(id, 10);
		this.log.info("CORE", "Vehicle received ID " + this.ID + ".");
	}

	/**
	 * Request all possible waypoints from RaceCarManager over REST
	 */
	private void requestWaypoints()
	{
		Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>()
		{
		}.getType();
		this.wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
		assert this.wayPoints != null;
		for (WayPoint wayPoint : this.wayPoints.values())
		{
			this.log.info("CORE", "Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
		}
		this.log.info("CORE", "All possible waypoints(" + this.wayPoints.size() + ") received.");
	}

	/**
	 * Sends starting point to the vehicle's SimKernel/RosKernel over socket connection.
	 */
	private void sendStartPoint()
	{
		this.log.info("CORE", "Starting point set as waypoint with ID " + startPoint + ".");
		if (!this.debugWithoutRosKernel)
			this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startPoint", this.wayPoints.get(this.startPoint)));
	}


	/**
	 * Interfaced method to parse MQTT message and topic after MQTT callback is triggered by incoming message.
	 * Used by messages coming from RacecarBackend to request a route job or a cost calculation request.
	 *
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	public void parseMQTT(String topic, String message)
	{
		this.log.info("NAVIGATOR", "received MQTT message: " + message);

		if (topic.matches("racecar/[0-9]+/job"))
		{
			this.navigator.handleJobRequest(message);
		}
		else if (topic.matches("racecar/[0-9]+/costrequest"))
		{
			String[] wayPointStringValues = message.split(" ");
			try
			{
				long[] wayPointValues = new long[wayPointStringValues.length];
				for (int index = 0; index < wayPointStringValues.length; index++)
				{

					wayPointValues[index] = Integer.parseInt(wayPointStringValues[index]);
				}
				this.weightManager.costRequest(wayPointValues);
			}
			catch (NumberFormatException e)
			{
				this.log.warning("CORE", "Parsing MQTT gives bad result: " + e);
			}
		}
		else if (topic.matches("racecar/[0-9]+/changeMap"))
		{
			this.mapManager.requestMap();
		}
	}

	/**
	 * Interfaced method to parse TCP message socket callback is triggered by incoming message.
	 * Used for messages about percentage route completion, next waypoint arrived, initial connection setup,
	 * stopping/killing/starting/restarting vehicle, setting the startpoint or cost calculation response.
	 *
	 * @param message received TCP socket message string
	 * @return a return answer to be send back over the socket to the SimKernel/RosKernel
	 */
	public String parseTCP(String message)
	{
		if (JSONUtils.isJSONValid(message))
		{
			//parses keyword to do the correct function call.
			switch (JSONUtils.getFirst(message))
			{
				case "percentage":
					this.navigator.locationUpdate((Location) JSONUtils.getObjectWithKeyWord(message, Location.class));
					break;
				case "arrivedWaypoint":
					this.navigator.wayPointReached();
					break;
				case "connect":
					this.connectReceive();
					break;
				case "kill":
					this.killCar();
					break;
				case "stop":
					this.sendAvailability(false);
					break;
				case "start":
					this.sendAvailability(true);
					break;
				case "startpoint":
					this.startPoint = (long) JSONUtils.getObjectWithKeyWord(message, Long.class);
					this.mqttUtils.publishMessage("racecar/" + ID + "/locationupdate", Long.toString((Long) JSONUtils.getObjectWithKeyWord(message, Long.class)));
					this.log.info("CORE", "Setting new starting point with ID " + JSONUtils.getObjectWithKeyWord(message, Long.class));
					break;
				case "restart":
					this.sendAvailability(true);
					this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentPosition", this.wayPoints.get(Core.startPoint)));
					this.log.info("CORE", "Vehicle restarted.");
					break;
				case "cost":
					this.weightManager.costComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
					break;
				case "costtiming":
					this.timeComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
					break;
				case "location":
					//this.log.info("CORE", "Car is at coordinates: " + (String) JSONUtils.getObjectWithKeyWord(message, String.class));
					// the current location is published but is not useful for the smartcityproject, the percentage updates are used
					break;
				default:
					this.log.warning("CORE", "No matching keyword when parsing JSON from Sockets. Data: " + message);
					break;
			}
		}
		return null;
	}

	/**
	 * Set availability status of vehicle in the RacecarBackend by sending MQTT message.
	 *
	 * @param state state to be send. (available=true, unavailable=false)
	 */
	private void sendAvailability(boolean state)
	{
		this.mqttUtils.publishMessage("racecar/" + ID + "/available", Boolean.toString(state));
		this.log.info("CORE", "Vehicle's availability status set to " + state + '.');
	}

	/**
	 * Closes all connections (TCP & MQTT), unregisters the vehicle with the RacecarBackend and shut the module down.
	 */
	private void killCar()
	{
		this.log.info("CORE", "Vehicle kill request. Closing connections and shutting down...");
		this.restUtils.getCall("delete/" + this.ID);
		if (!this.debugWithoutRosKernel) this.tcpUtils.closeTCP();
		{
			this.mqttUtils.closeMQTT();
		}
		System.exit(0);
	}


	/**
	 * When vehicle has completed cost calculation for the locationUpdate() function it sets the variables
	 * costCurrentToStartTiming and costStartToEndTiming of the Core to be used by locationUpdate().
	 *
	 * @param cost Cost object containing the weights of the sub-routes.
	 */
	private void timeComplete(Cost cost)
	{
		this.navigator.setCostCurrentToStartTiming(cost.getWeightToStart());
		this.navigator.setCostStartToEndTiming(cost.getWeight());
	}
}
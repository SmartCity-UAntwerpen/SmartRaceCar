package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.*;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Module representing the high-level of a vehicle.
 */
class Core implements TCPListener, MQTTListener
{

	// Help services
	private MQTTUtils mqttUtils;
	private TCPUtils tcpUtils;
	private RESTUtils restUtils;
	private Logger log;


	// Variables
	private long ID;                                                        	// ID given by RacecarBackend to identify vehicle.
	private HashMap<Long, WayPoint> wayPoints;                                	// Map of all loaded waypoints.
	private long startPoint;                                        			// Starting position on map. Given by Main argument.
	private boolean occupied;                                                	// To verify if racecar is currently occupied by a route job.

	// Subsystems
	private HeartbeatPublisher heartbeatPublisher;
	private Navigator navigator;
	private WeightManager weightManager;
	private CoreParameters params;
	private MapManager mapManager;


	/**
	 * Module representing the high-level of a vehicle.
	 *
	 * @param startPoint Starting point of the vehicle. Defined by input arguments of Main method.
	 * @param serverPort Port to listen for messages of SimKernel/Roskernel. Defined by input arguments of Main method.
	 * @param clientPort Port to send messages to SimKernel/Roskernel. Defined by input arguments of Main method.
	 */
	public Core(long startPoint, int serverPort, int clientPort, CoreParameters params) throws InterruptedException, IOException
	{
		String asciiArt1 = FigletFont.convertOneLine("SmartCity");
		System.out.println(asciiArt1);
		System.out.println("------------------------------------------------------------------");
		System.out.println("--------------------- F1 Racecar Core - v1.0 ---------------------");
		System.out.println("------------------------------------------------------------------");

		this.params = params;

		this.log = LoggerFactory.getLogger(Core.class);

		this.startPoint = startPoint;

		this.wayPoints = new HashMap<>();
		this.occupied = false;


		this.loadConfig();
		this.log.info("Current parameters: \n" + this.params.toString());

		this.log.info("Startup parameters: Starting Waypoint:" + startPoint + " | TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);

		this.log.info("Starting REST connection to " + this.params.getRESTCarmanagerURL());
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

        this.log.info("Starting MQTT connection on " + this.params.getMqttBroker());
		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic("racecar/" + ID + "/#");

		this.log.info("Connecting TCP on port " + clientPort + " and " + serverPort);
		this.tcpUtils = new TCPUtils(clientPort, serverPort, this);
		this.tcpUtils.start();

		if (!this.params.isDebug())
		{
			this.connectSend();
		}

		this.mapManager = new MapManager(this);
		this.log.info("Map manager started");

		if(!this.mapManager.requestMap())
		{
			// TODO find better mechanism to wait for download
			// map not downloaded yet -> waiting for download to finish
			this.log.info("Giving the map 10s to load.");
			Thread.sleep(10000); //10 seconds delay so the map can load before publishing the startpoint
		}


		this.sendStartPoint();

		this.heartbeatPublisher = new HeartbeatPublisher(new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this), this.ID);
		this.heartbeatPublisher.start();
		this.log.info("Heartbeat publisher was started.");

		this.navigator = new Navigator(this);
		this.log.info("Navigator was started.");

		this.weightManager = new WeightManager(this);
		this.log.info("Weightmanager was started");
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
		this.log.info("Requesting timing");

		List<Point> points = new ArrayList<>();
		points.add(this.wayPoints.get(wayPointIDs[0]));
		points.add(this.wayPoints.get(wayPointIDs[1]));
		if (!this.params.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("costtiming", points));
		} else
		{
			this.log.info("Debug mode -> timing = 5");
			this.weightManager.costCalculationComplete(new Cost(false, 5, 5, this.ID));
		}
	}

	/**
	 * Help method to load all configuration parameters from the properties file with the same name as the class.
	 * If it's not found then it will use the default ones.
	 */
	private void loadConfig()
	{
		CoreParameterParser parser = new CoreParameterParser();
		//this.params = parser.parse("/home/ubuntu/Git/SmartRacecar/SmartRaceCar/release/core.properties");
		this.params = parser.parse("core.properties");
	}


	/**
	 * Send connection request over sockets to RosKernel/SimKernel.
	 */
	private void connectSend()
	{
		this.log.info("Trying to connect to car");
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
	}

	/**
	 * Register vehicle with RacecarBackend over REST.
	 */
	private void register()
	{
		this.log.info("Registering vehicle");
		String id = this.restUtils.getTextPlain("register/" + Long.toString(this.startPoint));
		this.ID = Long.parseLong(id, 10);
		this.log.info("Vehicle received ID " + this.ID + ".");
	}

	/**
	 * Request all possible waypoints from RaceCarManager over REST
	 */
	private void requestWaypoints()
	{
		this.log.info("Requesting waypoints");
		Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>()
		{
		}.getType();
		this.wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getwaypoints"), typeOfHashMap);
		assert this.wayPoints != null;
		for (WayPoint wayPoint : this.wayPoints.values())
		{
			this.log.info("Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
		}
		this.log.info("All possible waypoints(" + this.wayPoints.size() + ") received.");
	}

	/**
	 * Sends starting point to the vehicle's SimKernel/RosKernel over socket connection.
	 */
	private void sendStartPoint()
	{
		this.log.info("Starting point set as waypoint with ID " + startPoint + ".");
		if (!this.params.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startPoint", this.wayPoints.get(this.startPoint)));
		}
		else
		{
			this.log.info("Debug mode -> not sending start point");
		}
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
		this.log.info("received MQTT message: " + message);

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
				this.log.warn("Parsing MQTT gives bad result: " + e);
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
					this.log.info("Connected to car.");
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
					this.log.info("Setting new starting point with ID " + JSONUtils.getObjectWithKeyWord(message, Long.class));
					break;
				case "restart":
					this.sendAvailability(true);
					this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentPosition", this.wayPoints.get(this.startPoint)));
					this.log.info("Vehicle restarted.");
					break;
				case "cost":
					this.weightManager.costCalculationComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
					break;
				case "costtiming":
					this.navigator.timingCalculationComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
					break;
				case "location":
					//this.log.info("CORE", "Car is at coordinates: " + (String) JSONUtils.getObjectWithKeyWord(message, String.class));
					// the current location is published but is not useful for the smartcityproject, the percentage updates are used
					break;
				default:
					this.log.warn("No matching keyword when parsing JSON from Sockets. Data: " + message);
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
		this.log.info("Vehicle's availability status set to " + state + '.');
	}

	/**
	 * Closes all connections (TCP & MQTT), unregisters the vehicle with the RacecarBackend and shut the module down.
	 */
	private void killCar()
	{
		this.log.info("Vehicle kill request. Closing connections and shutting down...");
		this.restUtils.getCall("delete/" + this.ID);
		if (!this.params.isDebug()) this.tcpUtils.closeTCP();
		{
			this.mqttUtils.closeMQTT();
		}
		System.exit(0);
	}
}
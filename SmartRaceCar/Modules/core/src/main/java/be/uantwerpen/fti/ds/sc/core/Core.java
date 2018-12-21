package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.core.Communication.BackendCommunicator;
import be.uantwerpen.fti.ds.sc.core.Communication.GeneralBackendCommunicator;
import be.uantwerpen.fti.ds.sc.core.Communication.VehicleCommunicator;
import be.uantwerpen.fti.ds.sc.core.Communication.GeneralVehicleCommunicator;
import com.github.lalyos.jfiglet.FigletFont;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;

/**
 * Module representing the high-level of a vehicle.
 */
class Core implements TCPListener, MQTTListener
{

	// Help services
	private MQTTUtils mqttUtils;
	private Logger log;


	// Variables
	private long ID;                                                        	// ID given by RacecarBackend to identify vehicle.
	private long startPoint;                                        			// Starting position on map. Given by Main argument.

	// Subsystems
	private HeartbeatPublisher heartbeatPublisher;
	private Navigator navigator;
	//private WeightManager weightManager;
	private CoreParameters params;
	private MapManager mapManager;

	// Communication
	private GeneralVehicleCommunicator vehicleCommunicator;
	private GeneralBackendCommunicator backendCommunicator;


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

		this.loadConfig();
		this.log.info("Current parameters: \n" + this.params.toString());

		this.log.info("Startup parameters: Starting Waypoint:" + startPoint + " | TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);

		BackendCommunicator backendCommunicator = new BackendCommunicator(this.params);
		this.backendCommunicator = backendCommunicator;

		HashMap<Long, WayPoint> wayPoints = new HashMap<>();
		wayPoints = this.backendCommunicator.requestWayPoints();
		this.ID = this.backendCommunicator.register(this.startPoint);

        this.log.info("Starting MQTT connection on " + this.params.getMqttBroker());
		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);

		VehicleCommunicator vehicleCommunicator = new VehicleCommunicator(this.params, this, clientPort, serverPort);
		this.vehicleCommunicator = vehicleCommunicator;
		this.vehicleCommunicator.start();

		if (!this.params.isDebug())
		{
			this.log.debug("Waiting 3 seconds before sending connect");
			Thread.sleep(3000);
			this.vehicleCommunicator.connect();
		}

		this.mapManager = new MapManager(this, this.params, backendCommunicator, vehicleCommunicator);
		this.log.info("Map manager started");

		if(!this.mapManager.configureMap())
		{
			// TODO find better mechanism to wait for download
			// map not downloaded yet -> waiting for download to finish
			this.log.info("Giving the map 10s to load.");
			Thread.sleep(10000); //10 seconds delay so the map can load before publishing the startpoint
		}

		this.navigator = new Navigator(this.ID, this.params, vehicleCommunicator, wayPoints);
		this.navigator.sendStartPoint(this.startPoint);

		this.heartbeatPublisher = new HeartbeatPublisher(new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this), this.ID);
		this.heartbeatPublisher.start();
		this.log.info("Heartbeat publisher was started.");


		this.log.info("Navigator was started.");

		//this.weightManager = new WeightManager(this);
		//this.log.info("Weightmanager was started");
	}

	public long getID()
	{
		return this.ID;
	}

	@Deprecated
	public CoreParameters getParams()
	{
		return this.params;
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
	 * Interfaced method to parse MQTT message and topic after MQTT callback is triggered by incoming message.
	 * Used by messages coming from RacecarBackend to request a route job or a cost calculation request.
	 *
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	public void parseMQTT(String topic, String message)
	{
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
					this.exit();
					break;
				case "stop":
					this.sendAvailability(false);
					this.log.info("Vehicle stopped");
					break;
				case "start":
					this.sendAvailability(true);
					this.log.info("Vehicle started");
					break;
				case "startpoint":
					this.startPoint = (long) JSONUtils.getObjectWithKeyWord(message, Long.class);
					//this.mqttUtils.publishMessage("racecar/" + ID + "/locationupdate", Long.toString((Long) JSONUtils.getObjectWithKeyWord(message, Long.class)));
					this.mqttUtils.publishMessage(this.params.getMqttTopic() + "/locationupdate/" + this.ID , Long.toString((Long) JSONUtils.getObjectWithKeyWord(message, Long.class)));
					this.log.info("Setting new starting point with ID " + JSONUtils.getObjectWithKeyWord(message, Long.class));
					break;
				case "restart":
					this.sendAvailability(true);
					//this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentPosition", this.wayPoints.get(this.startPoint)));
					this.navigator.sendCurrentPosition(this.startPoint);
					this.log.info("Vehicle restarted.");
					break;
				case "cost":
					//this.weightManager.costCalculationComplete((Cost) JSONUtils.getObjectWithKeyWord(message, Cost.class));
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
		//this.mqttUtils.publishMessage("racecar/" + ID + "/available", Boolean.toString(state));
		this.mqttUtils.publishMessage(this.params.getMqttTopic() + "/available/" + this.ID, Boolean.toString(state));
		this.log.info("Vehicle's availability status set to " + state + '.');
	}

	/**
	 * Closes all connections (TCP & MQTT), unregisters the vehicle with the RacecarBackend and shut the module down.
	 */
	private void exit()
	{
		this.log.info("Vehicle kill request. Closing connections and shutting down...");
		this.backendCommunicator.disconnect(this.ID);
		if (!this.params.isDebug())
		{
			this.vehicleCommunicator.disconnect();
			this.mqttUtils.closeMQTT();
		}
		System.exit(0);
	}
}
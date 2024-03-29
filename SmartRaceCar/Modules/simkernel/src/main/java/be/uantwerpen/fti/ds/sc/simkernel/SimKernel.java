package be.uantwerpen.fti.ds.sc.simkernel;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.*;
import be.uantwerpen.fti.ds.sc.simkernel.Communication.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Module that simulates the low level ROS element of the F1 car. It simulates all it's aspects.
 */
class SimKernel implements MessageListener
{
	private static final String CONFIG_NAME = "Simkernel.properties";

	private Configuration configuration;

	//Help services
	private CoreCommunication core;
	private ROSCommunication ROS;
	private SimDeployerCommunication simdeployer;

	//variables
	private Logger log;

	private boolean connected = false; // To verify socket connection to vehicle's Core.
	private Map map; // The currently used map.
	private WayPoint startPoint; // The point where it's at the start. Received from the core at the start.
	private Point currentPosition; // The point where it's currently positioned at or closest to.
	private HashMap<ArrayList<Point>, Cost> calculatedCosts; //The costs between two waypoints that were previously calculated

	/**
	 * Module that simulates the low level ROS element of the F1 car. It simulates all it's aspects.
	 *
	 */
	public SimKernel(long simID, String propertyPath) throws InterruptedException, IOException
	{
		this.log = LoggerFactory.getLogger(SimKernel.class);
		this.calculatedCosts = new HashMap<>();

		this.loadConfig(propertyPath);

		TcpServerAspect tcpServerAspect = (TcpServerAspect) this.configuration.get(AspectType.TCP_SERVER);
		TcpClientAspect tcpClientAspect = (TcpClientAspect) this.configuration.get(AspectType.TCP_CLIENT);

		this.ROS = new ROSCommunicator(this.configuration);
		this.core = new CoreCommunicator(tcpServerAspect.getServerPort(), tcpClientAspect.getClientPort(), this);
		this.core.start();
		this.simdeployer = new SimDeployerCommunicator(this.configuration, simID, this);

		while (!this.connected)
		{
			this.log.warn("Waiting for connection with vehicle Core on port " + tcpServerAspect.getServerPort());
			Thread.sleep(1000);
		}
	}

	/**
	 * Help method to load all configuration parameters from the properties file with the same name as the class.
	 * If it's not found then it will use the default ones.
	 */
	private void loadConfig(String propertyPath)
	{
		this.configuration = new Configuration();
		this.configuration.add(AspectType.MQTT);
		this.configuration.add(AspectType.ROS);
		this.configuration.add(AspectType.TCP_SERVER);
		this.configuration.add(AspectType.TCP_CLIENT);
		this.configuration.load(propertyPath + SimKernel.CONFIG_NAME);
	}

	/**
	 * Interfaced method to parse TCP message socket callback is triggered by incoming message.
	 * Used for messages about cost and timing requests, initial startup connection, the startpoint and current map
	 * settings, new job request information (next waypoint) or an mandatory update on the current position.
	 *
	 * @param message Received TCP socket message string
	 */
	@Override
	public void notify(String message)
	{
		this.log.debug("Received message: " + message);
		if (JSONUtils.isJSONValid(message))
		{
			//parses keyword to do the correct function call.
			switch (JSONUtils.getFirst(message))
			{
				case TcpMessages.Core.COST_TIMING:
					Type typeOfPoints = new TypeToken<ArrayList<Point>>()
					{
					}.getType();
					this.log.debug("Cost request: " + message);
					calculateTiming((ArrayList<Point>) JSONUtils.getObjectWithKeyWord(message, typeOfPoints));
					break;

				case TcpMessages.Core.CONNECT:
					this.connectReceive();
					break;

				case TcpMessages.Core.START_POINT:
					this.startPoint = (WayPoint) JSONUtils.getObjectWithKeyWord(message, WayPoint.class);
					this.log.info("Startpoint set to " + this.startPoint.getX() + "," + this.startPoint.getY() + "," + this.startPoint.getZ() + "," + this.startPoint.getW() + ".");
					this.currentPosition = new Point(this.startPoint.getX(), this.startPoint.getY(), this.startPoint.getZ(), this.startPoint.getW());
					break;

				case TcpMessages.Core.CURRENT_MAP:
					this.map = (Map) JSONUtils.getObjectWithKeyWord(message, Map.class);
					this.calculatedCosts.clear();
					this.log.info("Map set to '" + this.map.getName() + "'.");
					break;

				case TcpMessages.Core.NEXT_WAYPOINT:
					Type typeOfWayPoint = new TypeToken<WayPoint>()
					{
					}.getType();
					jobRequest((WayPoint) JSONUtils.getObjectWithKeyWord(message, typeOfWayPoint));
					break;

				case TcpMessages.Core.CURRENT_POSITION:
					Type typeOfWayPoint2 = new TypeToken<WayPoint>()
					{
					}.getType();
					this.currentPosition = (WayPoint) JSONUtils.getObjectWithKeyWord(message, typeOfWayPoint2);
					this.log.info("Current position set to " + currentPosition.getX() + "," + currentPosition.getY() + "," + currentPosition.getZ() + "," + currentPosition.getW() + ".");
					break;

				case MqttMessages.Topics.SimDeployer.KILL:
					this.exit();

				default:
					this.log.warn("No matching keyword when parsing JSON from Sockets. Data: " + message);
					break;
			}
		}
	}

	/**
	 * Method called when a connection from the Core arrives. Sends a message back to the Core.
	 */
	private void connectReceive()
	{
		this.core.connect();
		this.connected = true;
		this.log.info("Connected to Core.");
	}


	/**
	 * Method called for when a job request is received from the core. It contains the next waypoint to drive to.
	 * Given we are dealing with a simulated vehicle a request will be made to the RosServer to calculate how long
	 * the actual driving would take to the next waypoint. Then it will use this estimated time to simulate the driving.
	 *
	 * @param nextPoint Coordinates of the next waypoint to drive to.
	 */
	private void jobRequest(WayPoint nextPoint)
	{
		this.log.info("Job request to drive to " + nextPoint.getX() + "," + nextPoint.getY() + "," + nextPoint.getZ() + "," + nextPoint.getW() + ".");
		Cost cost = new Cost(false, 5, 5, (long) 0);

		RosAspect aspect = (RosAspect) this.configuration.get(AspectType.ROS);

		if(!aspect.isRosDebug())
		{
			List<Point> points = new ArrayList<>();
			points.add(this.currentPosition);
			points.add(this.currentPosition);
			Point next = new Point(nextPoint.getX(), nextPoint.getY(), nextPoint.getZ(), nextPoint.getW());
			points.add(next);
			Type typeOfCost = new TypeToken<Cost>()
			{
			}.getType();

			try
			{
				cost = this.ROS.requestCost(points, typeOfCost);
			}
			catch (IOException ioe)
			{
				this.log.error("An IOException was thrown while trying to calculate the cost to " + nextPoint.getID(), ioe);
				return;
			}
		}
		this.log.info("Travel time to destination is " + cost.getWeight() + "s.");
		if (cost.getWeight() != 0)
		{
			this.core.percentageUpdate(cost);
		}
		this.core.wayPointReached();
		this.currentPosition = new Point(nextPoint.getX(), nextPoint.getY(), nextPoint.getZ(), nextPoint.getW());
	}

	/**
	 * Method called for when a timing calculation request is received from the core. It contains a list of waypoints(2)
	 * to calculate the cost between. This is as simulated vehicle so it can't calculate these timings itself.
	 * It will do a REST request to the RosServer to calculate the estimated times between the current position
	 * and the starting location of the route, and between that starting location and the end location of the route.
	 *
	 * @param points List of the Point object class containing the 2 points to calculate the timings. (starting and end)
	 */
	private void calculateTiming(ArrayList<Point> points)
	{
		Cost cost = this.calcWeightROS(points);
		this.log.info("Calculated timing between current and start: " + cost.getWeightToStart() + "s. Timing to end : " + cost.getWeight() + "s.");
		this.core.sendTiming(cost);
	}

	/**
	 * This method is used by calculateCost en calculateTiming to request the weight between waypoints from a Ros server.
	 * If the same request was made previously, the cost will be returned from a local list.
	 *
	 * @param points List of the Point object class containing the 2 points to calculate the timings. (starting and end)
	 * @return object of the Cost class which contains the calculated costs
	 */
	private Cost calcWeightROS(ArrayList<Point> points)
	{
		ArrayList<Point> allPoints = new ArrayList<>();
		allPoints.add(this.currentPosition);
		allPoints.addAll(points);
		this.log.info("Cost request received. Requesting calculation from ROS Server.");
		Cost cost = new Cost(false, 5, 5, (long) 0);

		RosAspect rosAspect = (RosAspect) this.configuration.get(AspectType.ROS);
		if (!rosAspect.isRosDebug())
		{
			for (ArrayList<Point> list : this.calculatedCosts.keySet())
			{ // The containsKey method from the HashMap class doesn't correctly compare two ArrayLists
				if (list.equals(allPoints))
				{
					allPoints = list;
					break;
				}
			}

			if (this.calculatedCosts.containsKey(allPoints))
			{ //a request to the ROSkernel is intensive and needs to be avoided if possible
				this.log.info("Loaded cost locally from previously calculated costs");
				cost = this.calculatedCosts.get(allPoints);
			}
			else
			{
				Type typeOfCost = new TypeToken<Cost>()
				{
				}.getType();

				try
				{
					cost = this.ROS.requestCost(points, typeOfCost);
				}
				catch (IOException ioe)
				{
					this.log.error("An IOException was thrown while trying to calculate the cost to " + allPoints.toString(), ioe);
					return cost;
				}

				this.calculatedCosts.put(allPoints, cost);
				this.log.info("Cost was requested from ROSserver and added to local costs.");
			}
		}
		return cost;
	}

	public void exit()
	{
		this.core.exit();
		this.core.disconnect();

		System.exit(0);
	}
}

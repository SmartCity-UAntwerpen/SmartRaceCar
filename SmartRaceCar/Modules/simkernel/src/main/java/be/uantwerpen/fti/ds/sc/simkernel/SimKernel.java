package be.uantwerpen.fti.ds.sc.simkernel;

import be.uantwerpen.fti.ds.sc.common.*;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Type;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Properties;
import java.util.logging.Level;

/**
 * Module that simulates the low level ROS element of the F1 car. It simulates all it's aspects.
 */
class SimKernel implements TCPListener
{
	private static final String DEFAULT_CONFIG_PATH = "Simkernel.properties";

	private SimkernelParameters parameters;

	//Help services
	private TCPUtils tcpUtils;
	private RESTUtils restUtils;

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
	 * @param serverPort Port to listen for messages of  Core. Defined by input arguments of main method.
	 * @param clientPort Port to send messages to Core. Defined by input arguments of main method.
	 */
	public SimKernel(int serverPort, int clientPort) throws InterruptedException, IOException
	{
		this.log = LoggerFactory.getLogger(SimKernel.class);

		this.loadConfig();

		this.log.info("Startup parameters: TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
		this.restUtils = new RESTUtils(this.parameters.getROSServerURL());
		this.tcpUtils = new TCPUtils(clientPort, serverPort, this);
		this.tcpUtils.start();
		this.calculatedCosts = new HashMap<>();

		while (!this.connected)
		{
			this.log.warn("Waiting for connection with vehicle Core on port " + serverPort);
			Thread.sleep(1000);
		}
	}

	/**
	 * Help method to load all configuration parameters from the properties file with the same name as the class.
	 * If it's not found then it will use the default ones.
	 */
	private void loadConfig()
	{
		SimkernelParameterParser parser = new SimkernelParameterParser();
		this.parameters = parser.parse(SimKernel.DEFAULT_CONFIG_PATH);
	}

	/**
	 * Interfaced method to parse TCP message socket callback is triggered by incoming message.
	 * Used for messages about cost and timing requests, initial startup connection, the startpoint and current map
	 * settings, new job request information (next waypoint) or an mandatory update on the current position.
	 *
	 * @param message Received TCP socket message string
	 * @return A return answer to be send back over the socket to the Core.
	 */
	@Override
	public String parseTCP(String message)
	{
		if (JSONUtils.isJSONValid(message))
		{
			//parses keyword to do the correct function call.
			switch (JSONUtils.getFirst(message))
			{
				case "cost":
					Type typeOfPoints = new TypeToken<ArrayList<Point>>()
					{
					}.getType();
					calculateCost((ArrayList<Point>) JSONUtils.getObjectWithKeyWord(message, typeOfPoints));
					break;

				case "costtiming":
					Type typeOfPointss = new TypeToken<ArrayList<Point>>()
					{
					}.getType();
					calculateTiming((ArrayList<Point>) JSONUtils.getObjectWithKeyWord(message, typeOfPointss));
					break;

				case "connect":
					connectReceive();
					break;

				case "startPoint":
					this.startPoint = (WayPoint) JSONUtils.getObjectWithKeyWord(message, WayPoint.class);
					this.log.info("Startpoint set to " + startPoint.getX() + "," + startPoint.getY() + "," + startPoint.getZ() + "," + startPoint.getW() + ".");
					this.currentPosition = new Point(startPoint.getX(), startPoint.getY(), startPoint.getZ(), startPoint.getW());
					break;

				case "currentMap":
					this.map = (Map) JSONUtils.getObjectWithKeyWord(message, Map.class);
					this.calculatedCosts.clear();
					this.log.info("Map set to '" + this.map.getName() + "'.");
					break;

				case "nextWayPoint":
					Type typeOfWayPoint = new TypeToken<WayPoint>()
					{
					}.getType();
					jobRequest((WayPoint) JSONUtils.getObjectWithKeyWord(message, typeOfWayPoint));
					break;

				case "currentPosition":
					Type typeOfWayPoint2 = new TypeToken<WayPoint>()
					{
					}.getType();
					this.currentPosition = (WayPoint) JSONUtils.getObjectWithKeyWord(message, typeOfWayPoint2);
					this.log.info("Current position set to " + currentPosition.getX() + "," + currentPosition.getY() + "," + currentPosition.getZ() + "," + currentPosition.getW() + ".");
					break;

				default:
					this.log.warn("No matching keyword when parsing JSON from Sockets. Data: " + message);
					break;
			}
		}
		return null;
	}

	/**
	 * Method called when a connection from the Core arrives. Sends a message back to the Core.
	 */
	private void connectReceive()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
		this.connected = true;
		this.log.info("Connected to Core.");
	}

	/**
	 * Method used for sending the Core the message that a waypoint has been reached.
	 */
	private void wayPointReached()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("arrivedWaypoint"));
		this.connected = true;
		this.log.info("Arrived at waypoint. Waiting for next order.");
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
		if (!this.parameters.isROSServerDisabled())
		{
			List<Point> points = new ArrayList<>();
			points.add(this.currentPosition);
			points.add(this.currentPosition);
			points.add(nextPoint);
			Type typeOfCost = new TypeToken<Cost>()
			{
			}.getType();
			cost = (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.postJSONGetJSON("calcWeight", JSONUtils.arrayToJSONString(points)), typeOfCost);
		}
		this.log.info("Travel time to destination is " + cost.getWeight() + "s.");
		if (cost.getWeight() != 0)
		{
			for (int i = 0; i <= 20; i++)
			{
				try
				{
					Thread.sleep((cost.getWeight() * 1000) / 20);
					Location location = new Location(0, 0, 0, i * 5);
					this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("percentage", location));
					this.log.info("travelled " + i * 5 + "% of total route.");
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}
		wayPointReached();
		this.currentPosition = new Point(nextPoint.getX(), nextPoint.getY(), nextPoint.getZ(), nextPoint.getW());
	}

	/**
	 * Method called for when a cost calculation request is received from the core. It contains a list of waypoints(2)
	 * to calculate the cost between. This is as simulated vehicle so it can't calculate this cost itself. It will do
	 * a REST request to the RosServer to calculate the estimated times between the current position and the starting
	 * location of the route, and between that starting location and the end location of the route.
	 *
	 * @param points List of the Point object class containing the 2 points to calculate the weights. (starting and end)
	 */
	private void calculateCost(ArrayList<Point> points)
	{
		Cost cost = calcWeightROS(points);
		this.log.info("Calculated cost between current and start: " + cost.getWeightToStart() + "s. Cost to end : " + cost.getWeight() + "s.");
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("cost", cost));
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
		Cost cost = calcWeightROS(points);
		this.log.info("Calculated timing between current and start: " + cost.getWeightToStart() + "s. Timing to end : " + cost.getWeight() + "s.");
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("costtiming", cost));
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
		if (!this.parameters.isROSServerDisabled())
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
				cost = (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.postJSONGetJSON("calcWeight", JSONUtils.arrayToJSONString(allPoints)), typeOfCost);
				this.calculatedCosts.put(allPoints, cost);
				this.log.info("Cost was requested from ROSserver and added to local costs.");
			}
		}
		return cost;
	}
}

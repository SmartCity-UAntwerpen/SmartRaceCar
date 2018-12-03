package be.uantwerpen.fti.ds.sc.smartracecar.simkernel;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.github.lalyos.jfiglet.FigletFont;
import com.google.gson.reflect.TypeToken;

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
	//Standard settings (without config file loaded)
	private boolean debugWithoutRosServer = false; // debug parameter for using this module without a connected RosSever.
	private String restURL = "http://smartcity.ddns.net:8084"; // REST Service URL to RosServer

	//Help services
	private TCPUtils tcpUtils;
	private RESTUtils restUtils;

	//variables
	private Log oldLog; // logging instance
	private LogbackWrapper log;

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
	private SimKernel(int serverPort, int clientPort) throws InterruptedException, IOException
	{
		String asciiArt1 = FigletFont.convertOneLine("SmartCity");
		System.out.println(asciiArt1);
		System.out.println("-------------------------------------------------------------------");
		System.out.println("------------------ F1 Racecar SimKernel - v1.0 --------------------");
		System.out.println("-------------------------------------------------------------------");
		this.log = new LogbackWrapper();

		loadConfig();
		this.log.info("SIMKERNEL", "Startup parameters: TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
		this.restUtils = new RESTUtils(restURL);
		this.tcpUtils = new TCPUtils(clientPort, serverPort, this);
		this.tcpUtils.start();
		this.calculatedCosts = new HashMap<>();

		while (!this.connected)
		{
			this.log.warning("SIMKERNEL", "Waiting for connection with vehicle Core on port " + serverPort);
			Thread.sleep(1000);
		}
	}

	/**
	 * Help method to load all configuration parameters from the properties file with the same name as the class.
	 * If it's not found then it will use the default ones.
	 */
	@SuppressWarnings("Duplicates")
	private void loadConfig()
	{
		Properties prop = new Properties();
		InputStream input = null;
		try
		{
			String path = SimKernel.class.getProtectionDomain().getCodeSource().getLocation().getPath();
			String decodedPath = URLDecoder.decode(path, "UTF-8");
			decodedPath = decodedPath.replace("SimKernel.jar", "");
			input = new FileInputStream(decodedPath + "/simkernel.properties");
			prop.load(input);
			String debugLevel = prop.getProperty("debugLevel");

			switch (debugLevel)
			{
				case "debug":
					oldLog = new Log(this.getClass(), Level.CONFIG);
					break;
				case "info":
					oldLog = new Log(this.getClass(), Level.INFO);
					break;
				case "warning":
					oldLog = new Log(this.getClass(), Level.WARNING);
					break;
				case "severe":
					oldLog = new Log(this.getClass(), Level.SEVERE);
					break;
			}

			this.restURL = prop.getProperty("restURL");
			this.debugWithoutRosServer = Boolean.parseBoolean(prop.getProperty("debugWithoutRosServer"));
			this.log.info("SIMKERNEL", "Config loaded");
		} catch (IOException ex)
		{
			oldLog = new Log(this.getClass(), Level.INFO);
			this.log.warning("SIMKERNEL", "Could not read config file. Loading default settings. " + ex);
		} finally
		{
			if (input != null)
			{
				try
				{
					input.close();
				} catch (IOException e)
				{
					this.log.warning("SIMKERNEL", "Could not close config file. Loading default settings. " + e);
				}
			}
		}
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
					this.log.info("SIMKERNEL", "Startpoint set to " + startPoint.getX() + "," + startPoint.getY() + "," + startPoint.getZ() + "," + startPoint.getW() + ".");
					this.currentPosition = new Point(startPoint.getX(), startPoint.getY(), startPoint.getZ(), startPoint.getW());
					break;
				case "currentMap":
					this.map = (Map) JSONUtils.getObjectWithKeyWord(message, Map.class);
					this.calculatedCosts.clear();
					this.log.info("SIMKERNEL", "Map set to '" + this.map.getName() + "'.");
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
					this.log.info("SIMKERNEL", "Current position set to " + currentPosition.getX() + "," + currentPosition.getY() + "," + currentPosition.getZ() + "," + currentPosition.getW() + ".");
					break;
				default:
					this.log.warning("SIMKERNEL", "No matching keyword when parsing JSON from Sockets. Data: " + message);
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
		this.log.info("SIMKERNEL", "Connected to Core.");
	}

	/**
	 * Method used for sending the Core the message that a waypoint has been reached.
	 */
	private void wayPointReached()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("arrivedWaypoint"));
		this.connected = true;
		this.log.info("SIMKERNEL", "Arrived at waypoint. Waiting for next order.");
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
		this.log.info("SIMKERNEL", "Job request to drive to " + nextPoint.getX() + "," + nextPoint.getY() + "," + nextPoint.getZ() + "," + nextPoint.getW() + ".");
		Cost cost = new Cost(false, 5, 5, (long) 0);
		if (!this.debugWithoutRosServer)
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
		this.log.info("SIMKERNEL", "Travel time to destination is " + cost.getWeight() + "s.");
		if (cost.getWeight() != 0)
		{
			for (int i = 0; i <= 20; i++)
			{
				try
				{
					Thread.sleep((cost.getWeight() * 1000) / 20);
					Location location = new Location(0, 0, 0, i * 5);
					this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("percentage", location));
					this.log.info("SIMKERNEL", "travelled " + i * 5 + "% of total route.");
				} catch (InterruptedException e)
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
		this.log.info("SIMKERNEL", "Calculated cost between current and start: " + cost.getWeightToStart() + "s. Cost to end : " + cost.getWeight() + "s.");
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
		this.log.info("SIMKERNEL", "Calculated timing between current and start: " + cost.getWeightToStart() + "s. Timing to end : " + cost.getWeight() + "s.");
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
		this.log.info("SIMKERNEL", "Cost request received. Requesting calculation from ROS Server.");
		Cost cost = new Cost(false, 5, 5, (long) 0);
		if (!this.debugWithoutRosServer)
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
				this.log.info("SIMKERNEL", "Loaded cost locally from previously calculated costs");
				cost = this.calculatedCosts.get(allPoints);
			} else
			{
				Type typeOfCost = new TypeToken<Cost>()
				{
				}.getType();
				cost = (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.postJSONGetJSON("calcWeight", JSONUtils.arrayToJSONString(allPoints)), typeOfCost);
				this.calculatedCosts.put(allPoints, cost);
				this.log.info("SIMKERNEL", "Cost was requested from ROSserver and added to local costs.");
			}
		}
		return cost;
	}

	/**
	 * Main method to run this class as a separate jar file
	 *
	 * @param args tcpclientport and tcpserverport required
	 * @throws Exception
	 */
	public static void main(String[] args) throws Exception
	{
		if (args.length != 2)
		{
			System.out.println("Need 2 arguments to run. Possible arguments: tcpclientport(int) tcpserverport(int)");
			//System.exit(0);
		} else if (args.length == 2)
		{
			int serverPort = 0;
			int clientPort = 0;

			if (!args[0].isEmpty())
			{
				serverPort = Integer.parseInt(args[0]);
			}
			if (!args[1].isEmpty())
			{
				clientPort = Integer.parseInt(args[1]);
			}

			final SimKernel simKernel = new SimKernel(serverPort, clientPort);
		}
	}
}

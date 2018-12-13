package be.uantwerpen.fti.ds.sc.core;


import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Location;
import be.uantwerpen.fti.ds.sc.common.LogbackWrapper;
import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.LinkedList;
import java.util.Queue;

public class Navigator
{
	private Core core;

	private int costCurrentToStartTiming;                                    // Time in seconds from current position to start position of route.
	private int costStartToEndTiming;                                        // Time in seconds from start position to end position of route.

	private Queue<Long> currentRoute;                                        // All waypoint IDs to be handled in the current route.
	private int routeSize;                                                    // Current route's size.
	private LogbackWrapper log;

	public Navigator(Core core)
	{
		this.log = new LogbackWrapper(Navigator.class);

		this.core = core;

		this.costCurrentToStartTiming = -1;
		this.costStartToEndTiming = -1;
		this.routeSize = -1;

		this.currentRoute = new LinkedList<>();
	}

	public void setCostCurrentToStartTiming(int costCurrentToStartTiming)
	{
		this.costCurrentToStartTiming = costCurrentToStartTiming;
	}

	public void setCostStartToEndTiming(int costStartToEndTiming)
	{
		this.costStartToEndTiming = costStartToEndTiming;
	}

	/**
	 * Parses and handles MQTT messages concerning job requests.
	 *
	 * @param message
	 */
	public void handleJobRequest(String message)
	{
		if (message.equals("stop"))
		{
			sendWheelStates(0, 0);
		} else
		{
			if (!this.core.isOccupied())
			{
				String[] wayPointStringValues = message.split(" ");
				try
				{
					long[] wayPointValues = new long[wayPointStringValues.length];
					for (int index = 0; index < wayPointStringValues.length; index++)
					{
						wayPointValues[index] = Integer.parseInt(wayPointStringValues[index]);
					}
					this.jobRequest(wayPointValues);
				} catch (NumberFormatException e)
				{
					this.log.warning("NAVIGATOR", "Parsing MQTT gives bad result: " + e);
				}
			} else
			{
				this.log.warning("NAVIGATOR", "Current Route not completed. Not adding waypoints.");
				this.routeNotComplete();
			}
		}
	}

	/**
	 * Event call over interface for when MQTT connection receives new route job requests from the RacecarBackend.
	 * Adds all requested waypoints to route queue one by one.
	 * Sets the vehicle to occupied. Ignores the request if vehicle is already occupied.
	 * Then triggers the first waypoint to be send to the RosKernel/SimKernel.
	 *
	 * @param wayPointIDs Array of waypoint ID's that are on the route to be completed.
	 */
	public void jobRequest(long[] wayPointIDs)
	{
		this.log.info("NAVIGATOR", "Route request received.");
		Boolean error = false;
		if (!this.core.isOccupied())
		{
			this.core.setOccupied(true);
			this.core.timeRequest(wayPointIDs);
			while (this.costCurrentToStartTiming == -1 && this.costStartToEndTiming == -1)
			{
				this.log.info("NAVIGATOR", "Waiting for cost calculation");
				try
				{
					Thread.sleep(1000);
				} catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
			for (long wayPointID : wayPointIDs)
			{
				if (this.core.getWayPoints().containsKey(wayPointID))
				{
					this.currentRoute.add(wayPointID);
					this.log.info("NAVIGATOR", "Added waypoint with ID " + wayPointID + " to route.");
				} else
				{
					this.log.warning("NAVIGATOR", "Waypoint with ID '" + wayPointID + "' not found.");
					this.currentRoute.clear();
					error = true;
				}
			}
			if (!error)
			{
				this.routeSize = this.currentRoute.size();
				this.log.info("NAVIGATOR", "All waypoints(" + this.routeSize + ") of route added. Starting route.");
				this.updateRoute();
			} else
			{
				this.log.warning("NAVIGATOR", "Certain waypoints not found. Route cancelled.");
				this.routeError();
			}
		} else
		{
			this.log.warning("NAVIGATOR", "Current Route not completed. Not adding waypoints.");
			routeNotComplete();
		}
	}

	/**
	 * Event call over interface to be used when socket connection received message that waypoint has been reached.
	 */
	public void wayPointReached()
	{
		this.log.info("NAVIGATOR", "Waypoint reached.");
		updateRoute();
	}

	/**
	 * When vehicle's RosKernel/SimKernel sends update on route completion it needs to be transformed to the completion amount for the total route.
	 * To do this it receives a cost calculation from the RosServer that it requested and uses this to determine the size of each waypoint's sub-route.
	 *
	 * @param location Location object containing the current percentage value.
	 */
	public void locationUpdate(Location location)
	{
		if (this.currentRoute.size() == 0)
		{
			float weight = (float) this.costStartToEndTiming / (float) (this.costCurrentToStartTiming + this.costStartToEndTiming);
			location.setPercentage(Math.round((1 - weight) * 100 + location.getPercentage() * weight));
		} else if (this.currentRoute.size() == 1)
		{
			float weight = (float) this.costCurrentToStartTiming / (float) (this.costCurrentToStartTiming + this.costStartToEndTiming);
			location.setPercentage(Math.round(location.getPercentage() * weight));
		}
		this.log.info("NAVIGATOR", "Location Updated. Vehicle has " + location.getPercentage() + "% of route completed");
		this.core.getMqttUtils().publishMessage("racecar/" + this.core.getID() + "/percentage", JSONUtils.objectToJSONString(location));
	}

	/**
	 * When all a requested route job can't be done by the vehicle as it's still completing a route.
	 * Sends MQTT message to RacecarBackend to update the route status.
	 */
	private void routeNotComplete()
	{
		this.core.setOccupied(false);
		this.core.getMqttUtils().publishMessage("racecar/" + this.core.getID() + "/route", "notcomplete");
	}

	/**
	 * When all a requested route job can't be done by the vehicle.
	 * Sends MQTT message to RacecarBackend to update the route status.
	 */
	private void routeError()
	{
		this.log.warning("NAVIGATOR", "Route error. Route Cancelled");
		this.core.setOccupied(false);
		this.core.getMqttUtils().publishMessage("racecar/" + this.core.getID() + "/route", "error");
	}

	/**
	 * To be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
	 * Sends information of the next waypoint over the socket connection to the vehicle's SimKernel/RosKernel.
	 */
	private void updateRoute()
	{
		if (!this.currentRoute.isEmpty())
		{
			WayPoint nextWayPoint = this.core.getWayPoints().get(this.currentRoute.poll());
			if (!this.core.getParams().isDebug())
				this.core.getTcpUtils().sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("nextWayPoint", nextWayPoint));
			this.log.info("NAVIGATOR", "Sending next waypoint with ID " + nextWayPoint.getID() + " (" + (this.routeSize - this.currentRoute.size()) + "/" + this.routeSize + ")");
			if (this.core.getParams().isDebug())
			{ //Debug code to go over all waypoints with a 3s sleep in between.
				try
				{
					Thread.sleep(3000);
				} catch (InterruptedException e)
				{
					e.printStackTrace();
				}
				wayPointReached();
			}

		} else
		{
			routeCompleted();
		}
	}

	/**
	 * When all waypoints have been completed the vehicle becomes unoccupied again.
	 * Sends MQTT message to RacecarBackend to update the vehicle status.
	 */
	private void routeCompleted()
	{
		this.log.info("NAVIGATOR", "Route Completed.");
		this.core.setOccupied(false);
		this.core.getMqttUtils().publishMessage("racecar/" + this.core.getID() + "/route", "done");
	}

	/**
	 * Send wheel states to the vehicle over the socket connection. useful for emergency stops and other specific requests.
	 *
	 * @param throttle throttle value for the vehicle wheels.
	 * @param steer    rotation value for the vehicle wheels.
	 */
	private void sendWheelStates(float throttle, float steer)
	{
		if (!this.core.getParams().isDebug())
			this.core.getTcpUtils().sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("drive", new Drive(steer, throttle)));
		this.log.info("NAVIGATOR", "Sending wheel state Throttle:" + throttle + ", Steer:" + steer + ".");
	}


}

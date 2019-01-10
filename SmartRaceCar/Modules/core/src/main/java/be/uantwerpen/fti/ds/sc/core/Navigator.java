package be.uantwerpen.fti.ds.sc.core;


import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.KernelAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import be.uantwerpen.fti.ds.sc.core.Communication.NavigationBackendCommunication;
import be.uantwerpen.fti.ds.sc.core.Communication.NavigationVehicleCommunication;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class Navigator implements MQTTListener
{
	private long ID;
	private Configuration configuration;
	private Logger log;
	private MQTTUtils mqttUtils;

	private NavigationVehicleCommunication vehicle;

	private HashMap<Long, WayPoint> wayPoints;
	private int costCurrentToStartTiming;                                    // Time in seconds from current position to start position of route.
	private int costStartToEndTiming;                                        // Time in seconds from start position to end position of route.
	private boolean occupied;


	private Queue<Long> currentRoute;                                        // All waypoint IDs to be handled in the current route.
	private int routeSize;                                                   // Current route's size.


	public Navigator(Configuration configuration, NavigationVehicleCommunication vehicle, NavigationBackendCommunication backend)
	{
		this.log = LoggerFactory.getLogger(Navigator.class);

		this.configuration = configuration;

		this.wayPoints = backend.requestWayPoints();

		this.vehicle = vehicle;

		this.costCurrentToStartTiming = -1;
		this.costStartToEndTiming = -1;
		this.routeSize = -1;
		this.occupied = false;

		this.currentRoute = new LinkedList<>();
	}

	public void start(long ID)
	{
		this.ID = ID;
		try
		{

			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.JOB + "/" + this.ID);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to start MQTTUtils.", me);
		}
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
			this.vehicle.sendWheelStates(0, 0);
		}
		else
		{
			if (!this.occupied)
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
				}
				catch (NumberFormatException e)
				{
					this.log.warn("Parsing MQTT gives bad result: " + e);
				}
			}
			else
			{
				this.log.warn("Current Route not completed. Not adding waypoints.");
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
		this.log.info("Route request received.");

		boolean error = false;

		if (!this.occupied)
		{
			this.occupied = true;
			this.timeRequest(wayPointIDs);

			while (this.costCurrentToStartTiming == -1 && this.costStartToEndTiming == -1)
			{
				this.log.info("Waiting for cost calculation");
				try
				{
					Thread.sleep(1000);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}

			this.log.info("Cost calculation complete");

			for (long wayPointID : wayPointIDs)
			{
				if (this.wayPoints.containsKey(wayPointID))
				{
					this.currentRoute.add(wayPointID);
					this.log.info("Added waypoint with ID " + wayPointID + " to route.");
				}
				else
				{
					this.log.warn("Waypoint with ID '" + wayPointID + "' not found.");
					this.currentRoute.clear();
					error = true;
				}
			}

			if (!error)
			{
				this.routeSize = this.currentRoute.size();
				this.log.info("All waypoints(" + this.routeSize + ") of route added. Starting route.");
				this.updateRoute();
			}
			else
			{
				this.log.warn("Certain waypoints not found. Route cancelled.");
				this.routeError();
			}
		}
		else
		{
			this.log.warn("Current Route not completed. Not adding waypoints.");
			this.routeNotComplete();
		}
	}

	/**
	 * Event call over interface to be used when socket connection received message that waypoint has been reached.
	 */
	public void wayPointReached()
	{
		this.log.info("Waypoint reached.");
		this.locationUpdate();
		this.updateRoute();
	}

	/**
	 * When vehicle's RosKernel/SimKernel sends update on route completion it needs to be transformed to the completion amount for the total route.
	 * To do this it receives a cost calculation from the RosServer that it requested and uses this to determine the size of each waypoint's sub-route.
	 *
	 * @param location Location object containing the current percentage value.
	 */
	public void percentageUpdate(Location location)
	{
		if (this.currentRoute.size() == 0)
		{
			this.log.info("route size = 0");
			float weight = (float) this.costStartToEndTiming / (float) (this.costCurrentToStartTiming + this.costStartToEndTiming);
			location.setPercentage(Math.round((1 - weight) * 100 + location.getPercentage() * weight));
		}

		int percentage = location.getPercentage()/this.routeSize + (this.routeSize - this.currentRoute.size())*(100/this.routeSize);	// change percentage from percentage to next waypoint to percentage of route

		location.setPercentage(percentage);

		this.log.info("Location Updated. Vehicle has " + location.getPercentage() + "% of route completed");
		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.PERCENTAGE + "/" + this.ID, Integer.toString(location.getPercentage()));
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish progress update.", me);
		}
	}

	private void locationUpdate()
	{
		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.LOCATION_UPDATE + "/" + this.ID, Long.toString(this.currentRoute.poll()));
		}
		catch(MqttException mqttE)
		{
			this.log.warn("Location update failed: " + mqttE);
		}
	}

	/**
	 * When all a requested route job can't be done by the vehicle as it's still completing a route.
	 * Sends MQTT message to RacecarBackend to update the route status.
	 */
	private void routeNotComplete()
	{
		this.occupied = false;
		//this.mqttUtils.publish("racecar/" + this.core.getID() + "/route", "notcomplete");
		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE + "/" + this.ID, MqttMessages.Messages.Core.NOT_COMPLETE);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish route update.", me);
		}
	}

	/**
	 * When all a requested route job can't be done by the vehicle.
	 * Sends MQTT message to RacecarBackend to update the route status.
	 */
	private void routeError()
	{
		this.log.warn("Route error. Route Cancelled");
		this.occupied = false;
		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE + "/" + this.ID, MqttMessages.Messages.Core.ERROR);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish route update.", me);
		}
	}

	/**
	 * To be used when when a new waypoint has to be send to the vehicle or to check if route is completed.
	 * Sends information of the next waypoint over the socket connection to the vehicle's SimKernel/RosKernel.
	 */
	private void updateRoute()
	{
		if (!this.currentRoute.isEmpty())
		{
			WayPoint nextWayPoint = this.wayPoints.get(this.currentRoute.peek());

			this.log.info("Sending next waypoint with ID " + nextWayPoint.getID() + " (" + (this.routeSize - this.currentRoute.size()) + "/" + this.routeSize + ")");

			KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
			if (!kernelAspect.isDebug())
			{
				this.vehicle.sendNextWayPoint(nextWayPoint);
			}
			else
			{
				//Debug code to go over all waypoints with a 3s sleep in between.
				this.log.info("debug mode -> not sending waypoints to corelinker");
				try
				{
					Thread.sleep(3000);
				} catch (InterruptedException ie)
				{
					ie.printStackTrace();
				}
				this.wayPointReached();
			}
		}
		else
		{
			this.log.info("No waypoints left in route");
			this.routeCompleted();
		}
	}

	/**
	 * When all waypoints have been completed the vehicle becomes unoccupied again.
	 * Sends MQTT message to RacecarBackend to update the vehicle status.
	 */
	private void routeCompleted()
	{
		this.log.info("Route Completed.");
		this.occupied = false;
		//this.mqttUtils.publish("racecar/" + this.core.getID() + "/route", "done");

		try
		{MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE + "/" + this.ID, MqttMessages.Messages.Core.DONE);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to send route update.", me);
		}
	}

	/**
	 * When vehicle has completed cost calculation for the percentageUpdate() function it sets the variables
	 * costCurrentToStartTiming and costStartToEndTiming of the Core to be used by percentageUpdate().
	 *
	 * @param cost Cost object containing the weights of the sub-routes.
	 */
	public void timingCalculationComplete(Cost cost)
	{
		this.setCostCurrentToStartTiming(cost.getWeightToStart());
		this.setCostStartToEndTiming(cost.getWeight());
		this.log.info("Timing calculation complete");
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
		points.add(this.wayPoints.get(wayPointIDs[0]));
		points.add(this.wayPoints.get(wayPointIDs[1]));

		KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
		if (!kernelAspect.isDebug())
		{
			this.vehicle.timeRequest(points);
		}
		else
		{
			this.log.info("Debug mode -> timing = 5");
			this.timingCalculationComplete(new Cost(false, 5, 5, this.ID));
		}
	}

	public void sendCurrentPosition(long wayPoint)
	{
		this.vehicle.sendCurrentPosition(this.wayPoints.get(wayPoint));
	}

	public void sendStartPoint(long wayPoint)
	{
		this.vehicle.sendStartpoint(this.wayPoints.get(wayPoint));
	}


	@Override
	public void parseMQTT(String topic, String message)
	{
		this.handleJobRequest(message);
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.MessageQueueClient;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;

import javax.ws.rs.core.MediaType;
import java.util.*;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

@Controller
public class VehicleManager implements MQTTListener, VehicleRepository, OccupationRepository
{
	private static final long MQTT_DELIVERY_TIMEOUT = 30;

	private Logger log;
	private Configuration configuration;
	private MessageQueueClient messageQueueClient;
	private WaypointValidator waypointValidator;
	private Queue<Long> unusedIds;                      // This set contains all IDs of vehicles that were assigned once and then deleted
														// its a simple way to reuse IDs.

	AtomicLong currentId;

	private Map<Long, Boolean> occupation;

	/**
	 * Check if a vehicle with the given ID exists.
	 * @param vehicleId
	 * @return
	 */
	private boolean exists(long vehicleId)
	{
		return this.occupation.containsKey(vehicleId);
	}

	@Autowired
	public VehicleManager(@Qualifier("vehicleManager") Configuration configuration, WaypointValidator waypointValidator)
	{
		this.configuration = configuration;
		this.log = LoggerFactory.getLogger(this.getClass());

		this.currentId = new AtomicLong(0);

		this.log.info("Initializing Vehicle Manager...");

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MQTTUtils for VehicleManager.", me);
		}

		this.waypointValidator = waypointValidator;

		this.occupation = new HashMap<>();
		this.unusedIds = new LinkedList<>();

		this.log.info("Initialized Vehicle Manager.");
	}

	@Override
	public List<Long> getVehicleIds()
	{
		return new ArrayList<>(this.occupation.keySet());
	}

	@Override
	public int getNumVehicles()
	{
		return this.occupation.size();
	}

	/**
	 * Set a vehicle to be occupied or not occupied.
	 * @param vehicleId
	 * @param occupied	true if the vehicle should become occupied, false is the vehicle should become non-occupied.
	 * @return
	 */
	public void setOccupied(long vehicleId, boolean occupied)
	{
		if (!this.occupation.containsKey(vehicleId))
		{
			String errorString = "Tried to set non-existent vehicle's occupation to " + occupied + ", vehicle ID: " + vehicleId;
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		this.occupation.put(vehicleId, occupied);
	}

	@Override
	public boolean isOccupied(long vehicleId) throws NoSuchElementException
	{
		if (!this.occupation.containsKey(vehicleId))
		{
			String errorString = "Tried to check occupancy of vehicle " + vehicleId + ", but vehicle doesn't exist!";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		return this.occupation.get(vehicleId);
	}

	/*
	 *
	 *      REST Endpoints
	 *
	 */
	@RequestMapping(value="/carmanager/delete/{vehicleId}", method=RequestMethod.GET)
	public @ResponseBody ResponseEntity<String> delete(@PathVariable long vehicleId)
	{
		if (this.occupation.containsKey(vehicleId))
		{
			this.occupation.remove(vehicleId);

			try
			{
				MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
				this.messageQueueClient.publish(mqttAspect.getTopic() + "delete/" + vehicleId, "");
			}
			catch (Exception e)
			{
				this.log.error("Failed to publish vehicle deletion.", e);
			}

			this.log.info("Removing vehicle " + vehicleId);

			if (!this.unusedIds.contains(vehicleId))
			{
				this.unusedIds.add(vehicleId);
			}
			else
			{
				this.log.warn("Attempted to remove " + vehicleId + " but queue of unused IDs already contains the ID.");
			}

			return new ResponseEntity<>(HttpStatus.OK);
		}
		else
		{
			String errorString = "Got delete request for vehicle " + vehicleId + ", but vehicle doesn't exist.";
			this.log.warn(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}
	}

	@RequestMapping(value="/carmanager/register/{startWaypoint}", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> register(@PathVariable long startWaypoint)
	{
		if (!this.waypointValidator.exists(startWaypoint))
		{
			String errorString = "Tried to register vehicle with non-existent start id. (Start Waypoint: " + startWaypoint + ")";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		long newVehicleId = -1;

		if (!this.unusedIds.isEmpty())
		{
			newVehicleId = this.unusedIds.remove();
		}
		else
		{
			newVehicleId = this.currentId.getAndIncrement();
		}

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			MessageToken token = this.messageQueueClient.publish(mqttAspect.getTopic() + "register/" + newVehicleId, Long.toString(startWaypoint));
			token.waitForDelivery(MQTT_DELIVERY_TIMEOUT);
			this.messageQueueClient.publish(mqttAspect.getTopic() + "registered/" + newVehicleId, "done");
		}
		catch (Exception e)
		{
			this.log.error("Failed to publish vehicle registration.", e);
		}

		this.occupation.put(newVehicleId, false);

		this.log.info("Registered new vehicle (" + newVehicleId + "), Current Waypoint: " + startWaypoint);

		return new ResponseEntity<>(Long.toString(newVehicleId), HttpStatus.OK);
	}

	@RequestMapping(value="/carmanager/setOccupied/{vehicleId}/{occupied}", method=RequestMethod.POST)
	public @ResponseBody ResponseEntity<String> setOccupiedREST(@PathVariable long vehicleId, @PathVariable int occupied)
	{
		try
		{
			this.setOccupied(vehicleId, occupied != 0);
			return new ResponseEntity<>(HttpStatus.OK);
		}
		catch (NoSuchElementException nsee)
		{
			this.log.error("Failed to set vehicle occupation.", nsee);
			return new ResponseEntity<>(nsee.getMessage(), HttpStatus.NOT_FOUND);
		}
	}

	/*
	 *
	 *      MQTT Parsing
	 *
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.*;

import javax.ws.rs.core.MediaType;
import java.lang.reflect.Type;
import java.util.*;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Controller
public class VehicleManager implements MQTTListener
{
	private static class MQTTConstants
	{
		private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
		private static final Pattern AVAILABILITY_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/available");
		private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
		private static final Pattern HEARTBEAT_REGEX = Pattern.compile("racecar/[0-9]+/heartbeat");	//todo: move to Heartbeat checker
	}

	private static final Type LOCATION_TYPE = (new TypeToken<Location>()
	{
	}).getType();

	private BackendParameters parameters;
	private Logger log;
	private MQTTUtils mqttUtils;
	private RESTUtils MaaSRestUtils;
	private RESTUtils backboneRestUtils;
	private NavigationManager navigationManager;
	private MapManager mapManager;                  // Non-Owning reference to Map Manager
	private Map<Long, Vehicle> vehicles;

	private String getAvailabilityString(boolean available)
	{
		return available ? "Available" : "Not Available";
	}

	private boolean isPercentageUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.PERCENTAGE_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isAvailabilityUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.AVAILABILITY_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isRouteUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.ROUTE_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isHeartbeat(String topic)
	{
		Matcher matcher = MQTTConstants.HEARTBEAT_REGEX.matcher(topic);
		return matcher.matches();
	}

	private void updateRoute(long vehicleId, String mqttMessage)
	{
		switch (mqttMessage)
		{
			case "done":
				Vehicle vehicle = this.vehicles.get(vehicleId);
				vehicle.setOccupied(false);

				if (!this.parameters.isMaaSDisabled())
				{

					long jobId = vehicle.getJob().getIdJob();
					this.MaaSRestUtils.getTextPlain("completeJob/" + Long.toString(jobId));
				}

				vehicle.getLocation().setPercentage(100);

				this.log.info("Vehicle " + vehicleId + " completed its route.");

				break;

			case "error":
				this.vehicles.get(vehicleId).setOccupied(false);
				this.log.info("Vehicle " + vehicleId + " completed its route with errors.");
				break;

			case "notcomplete":
				this.vehicles.get(vehicleId).setOccupied(true);
				this.log.info("Vehicle " + vehicleId + " hasn't completed its route yet.");
				break;
		}
	}

	private void heartbeatUpdate(long vehicleId)
	{
		Date timestamp = new Date();
		this.vehicles.get(vehicleId).setHeartbeat(timestamp);
	}

	@Autowired
	public VehicleManager(@Qualifier("backend") BackendParameters parameters, MapManager mapManager)
	{
		this.parameters = parameters;
		this.log = LoggerFactory.getLogger(this.getClass());

		this.log.info("Setting up MQTT...");

		this.mqttUtils = new MQTTUtils(this.parameters.getMqttBroker(), this.parameters.getMqttUserName(), this.parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(this.parameters.getMqttTopic());

		this.log.info("Setting up MaaS REST Utils...");

		this.MaaSRestUtils = new RESTUtils(parameters.getRESTCarmanagerURL());

		this.log.info("Setting up Backbone REST Utils...");

		this.backboneRestUtils = new RESTUtils(parameters.getBackboneRESTURL());

		this.log.info("Starting Navigation Manager...");

		this.navigationManager = new NavigationManager(this, this.mapManager, parameters);

		this.log.info("Setting Map Manager...");

		this.mapManager = mapManager;

		this.vehicles = new HashMap<>();
	}

	/**
	 * Checks whether or not a vehicle existsOld.
	 *
	 * @param vehicleId The id of the vehicle to be checked
	 * @return
	 */
	public boolean exists(long vehicleId)
	{
		return this.vehicles.containsKey(vehicleId);
	}

	/**
	 * @param vehicleId
	 * @return
	 * @throws IndexOutOfBoundsException When a non-existent vehicle is queried, an exception is thrown
	 */
	public Vehicle get(long vehicleId)
	{
		if (this.exists(vehicleId))
		{
			return this.vehicles.get(vehicleId);
		}
		else
		{
			String errorString = "Tried to access vehicle that doesn't exist! (Id: " + vehicleId + ")";
			IndexOutOfBoundsException exception = new IndexOutOfBoundsException(errorString);
			this.log.error(errorString, exception);
			throw exception;
		}
	}

	/**
	 * Returns an iterator over a keyset of the vehicle ID's.
	 *
	 * @return
	 */
	public Iterator<Long> getIdIterator()
	{
		return this.vehicles.keySet().iterator();
	}

	public int getNumVehicles()
	{
		return this.vehicles.size();
	}

	/*
	 *
	 *      REST Endpoints
	 *
	 */
	@RequestMapping(value="/carmanager/delete/{id}", method= RequestMethod.GET)
	public @ResponseBody ResponseEntity<String> delete(@PathVariable long id)
	{
		if (this.vehicles.containsKey(id))
		{
			if (!this.parameters.isBackboneDisabled())
			{
				this.backboneRestUtils.getTextPlain("bot/delete/" + id);
			}

			this.vehicles.remove(id);

			this.log.info("Removing vehicle " + id);

			return new ResponseEntity<>(HttpStatus.OK);
		}
		else
		{
			String errorString = "Got delete request for vehicle " + id + ", but vehicle doesn't exist.";
			this.log.warn(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}
	}

	@RequestMapping(value="/carmanager/register/{startWaypoint}", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> register(@PathVariable long startWaypoint)
	{
		if (!this.mapManager.existsOld(startWaypoint))
		{
			String errorString = "Tried to register vehicle with non-existent start id. (Start Waypoint: " + startWaypoint + ")";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		long newVehicleId = -1;

		if (this.parameters.isBackboneDisabled())
		{
			newVehicleId = this.vehicles.size();
		}
		else
		{
			newVehicleId = Long.parseLong(this.backboneRestUtils.getJSON("bot/newBot/car"));
		}

		this.vehicles.put(newVehicleId, new Vehicle(newVehicleId, startWaypoint));

		this.log.info("Registered new vehicle (" + newVehicleId + "), Current Waypoint: " + startWaypoint);

		return new ResponseEntity<>(Long.toString(newVehicleId), HttpStatus.OK);
	}

	@RequestMapping(value="/carmanager/posAll", method=RequestMethod.GET, produces = MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> getPositions()
	{
		List<Location> locations = new ArrayList<>();

		for (Long vehicleId : this.vehicles.keySet())
		{
			Vehicle vehicle = this.vehicles.get(vehicleId);
			locations.add(vehicle.getLocation());
		}

		this.log.info("Request for all positions processed, returning " + Integer.toString(locations.size()) + " locations.");

		return new ResponseEntity<>(JSONUtils.arrayToJSONString(locations), HttpStatus.OK);
	}

	@RequestMapping(value="/carmanager/getVehicles", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> getVehicles()
	{
		return new ResponseEntity<>(JSONUtils.objectToJSONStringWithKeyWord("vehicles", this.vehicles), HttpStatus.OK);
	}

	/*
	 *
	 *      MQTT Parsing
	 *
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
		long id = TopicUtils.getCarId(topic);

		if ((id != -1) && (this.exists(id)))
		{
			if (this.isPercentageUpdate(topic))
			{
				//todo: Refactor JSON message to only have percentage and no other location information
				Location location = (Location) JSONUtils.getObject(message, LOCATION_TYPE);
				this.vehicles.get(id).getLocation().setPercentage(location.getPercentage());
				this.log.info("Received Percentage update for vehicle " + id + ", Status: " + location.getPercentage() + "%.");
			}
			else if (this.isAvailabilityUpdate(topic))
			{
				boolean availability = Boolean.parseBoolean(message);
				this.vehicles.get(id).setAvailable(availability);
				this.log.info("Received Availability update for vehicle " + id + ", Status: " + this.getAvailabilityString(availability));
			}
			else if (this.isRouteUpdate(topic))
			{
				this.log.info("Received Route Update for vehicle " + id + "");
				this.updateRoute(id, message);
			}
			else if (this.isHeartbeat(topic))
			{
				this.log.info("Received Heartbeat from " + id);
				this.heartbeatUpdate(id);
			}
		}
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.annotation.Lazy;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Cost Answers
@Controller
public class NavigationManager implements MQTTListener, LocationRepository
{
	private static class MQTTConstants
	{
		private static final Pattern COST_ANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
		private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
	}

	private static final Type COST_TYPE = (new TypeToken<Cost>()
	{
	}).getType();

	private Logger log;
	private MQTTUtils mqttUtils;
	private List<Cost> costList;
	private VehicleRepository vehicleRepository;
	private VehicleValidator vehicleValidator;
	private WaypointValidator waypointValidator;
	private java.util.Map<Long, Long> vehicleLocations;	// This map keeps track of the location of every vehicle
	// The key is the vehicleId, the value is the locationId

	private boolean isCostAnswer(String topic)
	{
		Matcher matcher = MQTTConstants.COST_ANSWER_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isLocationUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.LOCATION_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	public NavigationManager(@Lazy VehicleRepository vehicleRepository, VehicleValidator vehicleValidator, WaypointValidator waypointValidator, Parameters parameters)
	{
		this.log = LoggerFactory.getLogger(NavigationManager.class);

		this.log.info("Initializing Navigation Manager...");

		this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(parameters.getMqttTopic());

		this.costList = new ArrayList<>();
		this.vehicleLocations = new HashMap<>();
		this.vehicleRepository = vehicleRepository;
		this.vehicleValidator = vehicleValidator;
		this.waypointValidator = waypointValidator;

		this.log.info("Initialized Navigation Manager.");
	}

	/*
	 *
	 * 	REST ENDPOINTS
	 *
	 */

	@RequestMapping(value="/carmanager/posAll", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> getPositions()
	{
		List<Location> locations = new ArrayList<>();

		for (Long vehicleId : this.vehicleLocations.keySet())
		{
			Location location = new Location(vehicleId, this.vehicleLocations.get(vehicleId), this.vehicleLocations.get(vehicleId), 0);
			locations.add(location);
		}

		this.log.info("Request for all positions processed, returning " + locations.size() + " locations.");

		return new ResponseEntity<>(JSONUtils.arrayToJSONString(locations), HttpStatus.OK);
	}

	@Override
	public void setLocation(long vehicleId, long locationId)
	{
		this.log.info("Setting the location of vehicle " + vehicleId + " to " + locationId + ".");

		if (!this.waypointValidator.exists(locationId))
		{
			String errorString = "Tried to set location of " + vehicleId + " to a non-existent location: " + locationId;
			this.log.error(errorString);

			throw new IndexOutOfBoundsException (errorString);
		}

		this.vehicleLocations.put(vehicleId, locationId);
	}

	@Override
	public long getLocation(long vehicleId)
	{
		this.log.info("Fetching location for vehicle " + vehicleId + ".");

		if (!this.vehicleLocations.containsKey(vehicleId))
		{
			String errorString = "Vehicle " + vehicleId + " doesn't have a location.";
			this.log.error(errorString);

			throw new IndexOutOfBoundsException (errorString);
		}

		return this.vehicleLocations.get(vehicleId);
	}

	/**
	 * REST GET server service to get a calculation cost of all available vehicles. It requests from each vehicle a calculation
	 * of a possible route and returns a JSON containing all answers.
	 *
	 * @param startId Starting waypoint ID.
	 * @param endId   Ending waypoint ID.
	 * @return REST response of the type JSON containg all calculated costs of each vehicle.
	 */
	@Deprecated
	@RequestMapping(value = "calcWeight/{startId}/{endId}", method = RequestMethod.GET, produces = MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> calculateCostsRequest(@PathVariable("startId") long startId, @PathVariable("endId") long endId) throws InterruptedException
	{
		this.log.info("Received cost request for " + startId + " -> " + endId + ".");

		if (!this.waypointValidator.exists(startId))
		{
			String errorString = "Request cost with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		if (!this.waypointValidator.exists(endId))
		{
			String errorString = "Request cost with non-existent end waypoint " + endId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		if (this.vehicleRepository.getNumVehicles() == 0)
		{
			String errorString = "No vehicles exist.";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		int totalVehicles = 0;
		int timer = 0;

		for (long vehicleId: this.vehicleRepository.getVehicleIds())
		{
			Vehicle vehicle = this.vehicleRepository.get(vehicleId);

			if (vehicle.isAvailable())
			{
				totalVehicles++;
				this.mqttUtils.publishMessage("racecar/" + vehicle.getID() + "/costrequest", startId + " " + endId);
			}
		}

		// Runs for 100 iterations, each a little over 200ms
		while ((this.costList.size() < totalVehicles) && (timer != 100))
		{
			// Wait for each vehicle to complete the request or timeout after 100 attempts.
			this.log.info("waiting for vehicles to complete request.");
			Thread.sleep(200);
			timer++;
		}

		List<Cost> costCopy = new ArrayList<>();

		for (Cost cost : this.costList)
		{
			costCopy.add(cost.clone());
		}

		this.costList.clear();

		this.log.info("Cost calculation request completed.");

		return new ResponseEntity<>(JSONUtils.arrayToJSONString(costCopy), HttpStatus.OK);
	}

	/*
	 *
	 *      MQTT Parsing
	 *
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
		long vehicleId = TopicUtils.getCarId(topic);

		if (!this.vehicleValidator.exists(vehicleId))
		{
			this.log.warn("Received MQTT message from non-existent vehicle " + vehicleId);
			return;
		}

		// We received an MQTT cost answer
		if (this.isCostAnswer(topic))
		{
			Cost cost = (Cost) JSONUtils.getObject("value", COST_TYPE);
			this.costList.add(cost);
		}
		// We received an MQTT location update
		else if (this.isLocationUpdate(topic))
		{
			long locationId = Long.parseLong(message);
			this.vehicleLocations.put(vehicleId, locationId);
		}
	}

	public void removeVehicle(long vehicleId)
	{
		this.log.info("Removing vehicle " + vehicleId);

		if (this.vehicleLocations.containsKey(vehicleId))
		{
			this.vehicleLocations.remove(vehicleId);
		}
		else
		{
			String errorString = "Tried to remove non-existent vehicle (" + vehicleId + ").";
			this.log.error(errorString);
			throw new IndexOutOfBoundsException (errorString);
		}
	}
}
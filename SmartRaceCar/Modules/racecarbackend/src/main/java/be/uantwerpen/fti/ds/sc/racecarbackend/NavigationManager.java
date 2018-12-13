package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
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
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Cost Answers
@Controller
public class NavigationManager implements MQTTListener
{
	private static class MQTTConstants
	{
		private static final Pattern COST_ANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
		private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
	}

	private static final Type COST_TYPE = (new TypeToken<Cost>()
	{
	}).getType();

	private Parameters parameters;
	private Logger log;
	private MQTTUtils mqttUtils;
	private List<Cost> costList;
	private VehicleManager vehicleManager;
	private MapManager mapManager;

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

	public NavigationManager(VehicleManager vehicleManager, MapManager mapManager, Parameters parameters)
	{
		this.parameters = parameters;
		this.log = LoggerFactory.getLogger(NavigationManager.class);

		this.log.info("Setting up MQTT...");

		this.mqttUtils = new MQTTUtils(this.parameters.getMqttBroker(), this.parameters.getMqttUserName(), this.parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(this.parameters.getMqttTopic());

		this.log.info("Setting fields and creating cost list...");

		this.costList = new ArrayList<>();
		this.vehicleManager = vehicleManager;
		this.mapManager = mapManager;
	}

	/**
	 * REST GET server service to get a calculation cost of all available vehicles. It requests from each vehicle a calculation
	 * of a possible route and returns a JSON containing all answers.
	 *
	 * @param startId Starting waypoint ID.
	 * @param endId   Ending waypoint ID.
	 * @return REST response of the type JSON containg all calculated costs of each vehicle.
	 */
	@RequestMapping(value = "calcWeight/{startId}/{endId}", method = RequestMethod.GET, produces = MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> calculateCostsRequest(@PathVariable("startId") long startId, @PathVariable("endId") long endId) throws InterruptedException
	{
		if (!this.mapManager.exists(startId))
		{
			String errorString = "Request cost with non-existent start waypoint " + Long.toString(startId) + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		if (!this.mapManager.exists(endId))
		{
			String errorString = "Request cost with non-existent end waypoint " + Long.toString(endId) + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		if (this.vehicleManager.getNumVehicles() == 0)
		{
			String errorString = "No vehicles exist" + Long.toString(endId) + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		int totalVehicles = 0;
		int timer = 0;

		Iterator<Long> vehicleIdIterator = this.vehicleManager.getIdIterator();

		while (vehicleIdIterator.hasNext())
		{
			Long vehicleId = vehicleIdIterator.next();
			Vehicle vehicle = this.vehicleManager.get(vehicleId);

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
		long id = TopicUtils.getCarId(topic);

		// id == -1 means the topic wasn't valid
		// It's also possible that the topic was valid, but the vehicle just doesn't exist
		if ((id != -1) && (this.vehicleManager.exists(id)))
		{
			// We received an MQTT cost answer
			if (this.isCostAnswer(topic))
			{
				Cost cost = (Cost) JSONUtils.getObject("value", COST_TYPE);
				this.costList.add(cost);
			}
			// We received an MQTT location update
			else if (this.isLocationUpdate(topic))
			{
				try
				{
					long locationId = Long.parseLong(message);
					int percentage = this.vehicleManager.get(id).getLocation().getPercentage();
					Location location = new Location(id, locationId, locationId, percentage);
					this.vehicleManager.get(id).setLocation(location);
				}
				catch (Exception vehicleNotFoundException)
				{
					this.log.error("Tried to update location on non-existent vehicle.", vehicleNotFoundException);
				}
			}
		}
	}
}
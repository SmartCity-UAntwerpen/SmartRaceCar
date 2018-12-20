package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Controller;

import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Cost Answers
@Controller
public class NavigationManager implements MQTTListener, LocationRepository
{
	private static class MQTTConstants
	{
		private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
	}

	private Logger log;
	private MQTTUtils mqttUtils;
	private java.util.Map<Long, Long> vehicleLocations;
	// This map keeps track of the location of every vehicle
	// The key is the vehicleId, the value is the locationId

	private boolean isLocationUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.LOCATION_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	public NavigationManager(Parameters parameters)
	{
		this.log = LoggerFactory.getLogger(NavigationManager.class);

		this.log.info("Initializing Navigation Manager...");

		this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(parameters.getMqttTopic());

		this.vehicleLocations = new HashMap<>();

		this.log.info("Initialized Navigation Manager.");
	}

	@Override
	public void setLocation(long vehicleId, long locationId)
	{
		this.log.info("Setting the location of vehicle " + vehicleId + " to " + locationId + ".");
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

	/*
	 *
	 *      MQTT Parsing
	 *
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
		long vehicleId = TopicUtils.getCarId(topic);

		if (this.isLocationUpdate(topic))
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
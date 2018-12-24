package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Controller;

import java.util.HashMap;

// Route Update
// Cost Answers
@Controller
public class NavigationManager implements MQTTListener, LocationRepository
{
	private static final String MQTT_LOCATION_POSTFIX = "locationupdate/#";
	private static final String MQTT_REGISTER_POSTFIX = "register/#";
	private static final String MQTT_DELETE_POSTFIX = "delete/#";

	private Logger log;
	private Parameters parameters;
	private MQTTUtils mqttUtils;
	private java.util.Map<Long, Long> vehicleLocations;
	// This map keeps track of the location of every vehicle
	// The key is the vehicleId, the value is the locationId

	private boolean isDeletion(String topic)
	{
		// Remove the trailing '#' and check the topic
		String deleteTopic = MQTT_DELETE_POSTFIX.substring(0, MQTT_DELETE_POSTFIX.length() - 2);
		return topic.startsWith(this.parameters.getMqttTopic() + deleteTopic);
	}

	private void removeVehicle(long vehicleId)
	{
		this.log.info("Removing vehicle " + vehicleId + " from NavigationManager.");

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

	public NavigationManager(Parameters parameters)
	{
		this.log = LoggerFactory.getLogger(NavigationManager.class);
		this.parameters = parameters;

		this.log.info("Initializing Navigation Manager...");

		try
		{
			this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);
			this.mqttUtils.subscribe(parameters.getMqttTopic() + MQTT_LOCATION_POSTFIX);
			this.mqttUtils.subscribe(parameters.getMqttTopic() + MQTT_REGISTER_POSTFIX);
			this.mqttUtils.subscribe(parameters.getMqttTopic() + MQTT_DELETE_POSTFIX);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MQTTUtils for NavigationManager.", me);
		}

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

		if (this.isDeletion(topic))
		{
			this.removeVehicle(vehicleId);
		}
		else
		{
			// If the message isn't a deletion its a registration or a location update.
			// Parsing is the same either way.

			// The content of the location update and register messages is the same (The vehicle's location)
			// So we don't need any special parsing
			long locationId = Long.parseLong(message);

			this.vehicleLocations.put(vehicleId, locationId);
			this.log.info("Received location update from vehicle " + vehicleId + ", new location is " + locationId);
		}
	}
}
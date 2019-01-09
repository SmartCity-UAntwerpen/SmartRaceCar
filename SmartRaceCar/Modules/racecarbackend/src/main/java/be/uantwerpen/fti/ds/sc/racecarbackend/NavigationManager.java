package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.MessageQueueClient;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.HashMap;

// Route Update
// Cost Answers
@Service
public class NavigationManager implements MQTTListener, LocationRepository
{
	private Logger log;
	private Configuration configuration;
	private MessageQueueClient messageQueueClient;
	private java.util.Map<Long, Long> vehicleLocations;
	// This map keeps track of the location of every vehicle
	// The key is the vehicleId, the value is the locationId

	private boolean isDeletion(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		// Remove the trailing '#' and check the topic
		return topic.startsWith(mqttAspect.getTopic() + Messages.BACKEND.DELETE);
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

	public NavigationManager(@Qualifier("navigationManager") Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(NavigationManager.class);
		this.configuration = configuration;

		this.log.info("Initializing Navigation Manager...");

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + Messages.CORE.LOCATION_UPDATE + "/#");
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + Messages.BACKEND.REGISTER + "/#");
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + Messages.BACKEND.DELETE + "/#");
		}
		catch (Exception e)
		{
			this.log.error("Failed to set up MQTTUtils for NavigationManager.", e);
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
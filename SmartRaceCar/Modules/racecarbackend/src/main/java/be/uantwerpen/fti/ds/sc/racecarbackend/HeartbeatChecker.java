package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.MessageQueueClient;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.RacecarAspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.util.Date;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
@Component
class HeartbeatChecker implements MQTTListener
{
	private static final String MQTT_HEARTBEAT_POSTFIX = "heartbeat/#";
	private static final String MQTT_REGISTER_POSTFIX = "register/#";
	private static final String MQTT_DELETE_POSTFIX = "delete/#";

	@Value("${Racecar.Heartbeat.interval}")
	private long CHECK_INTERVAL;		// Interval between heartbeat checks (in s)

	@Value("${Racecar.Heartbeat.max_age}")
	private long MAX_DELTA;			    // Maximum amount of time between consecutive heartbeats (in ms)

	private Logger log;
	private Configuration configuration;
	private RESTUtils restUtils;
	private MessageQueueClient messageQueueClient;
	private MQTTUtils mqttUtils;
	private Map<Long, Date> heartbeats;

	private boolean isHeartbeat(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		// Remove the trailing '#' and compare the topic
		String heartbeatTopic = MQTT_HEARTBEAT_POSTFIX.substring(0, MQTT_HEARTBEAT_POSTFIX.length() - 2);
		return topic.startsWith(mqttAspect.getTopic() +  heartbeatTopic);
	}

	private boolean isRegistration(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		// Remove the trailing '#' and compare the topic
		String heartbeatTopic = MQTT_REGISTER_POSTFIX.substring(0, MQTT_REGISTER_POSTFIX.length() - 2);
		return topic.startsWith(mqttAspect.getTopic() +  heartbeatTopic);
	}

	private boolean isDeletion(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		// Remove the trailing '#' and check the topic
		String deleteTopic = MQTT_DELETE_POSTFIX.substring(0, MQTT_DELETE_POSTFIX.length() - 2);
		return topic.startsWith(mqttAspect.getTopic() +  deleteTopic);
	}

	private void updateHeartbeat(long vehicleId)
	{
		if (this.heartbeats.containsKey(vehicleId))
		{
			this.heartbeats.put(vehicleId, new Date());
		}
		else
		{
			this.log.warn("Received heartbeat for non-existent vehicle (" + vehicleId + ").");
		}
	}

	@Scheduled(fixedRateString="${Racecar.Heartbeat.interval}")
	private void checkBeats()
	{
		this.log.info("Checking the heartbeats of " + this.heartbeats.size() + " vehicles...");

		Date currentTime = new Date();

		for (long vehicleId : this.heartbeats.keySet())
		{
			final long delta = (currentTime.getTime() - this.heartbeats.get(vehicleId).getTime());

			this.log.debug("Vehicle " + vehicleId + "'s last heartbeat came " + (delta / 1000) + "s ago.");

			if (delta > MAX_DELTA) //longer than 90 seconds
			{
				this.restUtils.get("delete/" + vehicleId);
				this.log.warn("Vehicle with ID: " + vehicleId + " was removed since it hasn't responded for over 90s");
			}
		}

		this.log.info("All heartbeats were checked.");
	}

	/**
	 * Add a new vehicle to the heartbeat checker, taking the current time as its first heartbeat
	 *
	 * @param vehicleId The ID of the vehicle we want to check.
	 */
	public void addVehicle(long vehicleId)
	{
		this.heartbeats.put(vehicleId, new Date());
	}

	/**
	 * Remove a vehicle from the heartbeat checker, causing its heartbeats to not be checked anymore.
	 *
	 * @param vehicleId The vehicle to be removed.
	 */
	public void removeVehicle(long vehicleId)
	{
		if (this.heartbeats.containsKey(vehicleId))
		{
			this.log.info("Removing vehicle " + vehicleId + " from HeartbeatChecker.");
			this.heartbeats.remove(vehicleId);
		}
		else
		{
			this.log.warn("Tried to remove non-existent vehicle (" + vehicleId + ").");
		}
	}

	/**
	 * constructor for the HeartbeatChecker class
	 *
	 * @param parameters parameters used to start backend
	 */
	@Autowired
	public HeartbeatChecker(@Qualifier("heartbeatChecker") Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(HeartbeatChecker.class);
		this.configuration = configuration;

		this.log.debug("Initializing Heartbeat checker...");

		RacecarAspect racecarAspect = (RacecarAspect) configuration.get(AspectType.RACECAR);
		this.restUtils = new RESTUtils(racecarAspect.getRacecarServerUrl());

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + MQTT_REGISTER_POSTFIX);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + MQTT_DELETE_POSTFIX);
		}
		catch (Exception e)
		{
			this.log.error("Failed to start MessageQueueClient for HeartbeatChecker.", e);
		}

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + MQTT_HEARTBEAT_POSTFIX);
		}
		catch (Exception e)
		{
			this.log.error("Failed to start MQTTUtils for HeartbeatChecker.", e);
		}

		this.heartbeats = new ConcurrentHashMap<>();

		this.log.debug("Initialized Heartbeat checker.");
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		long vehicleId = TopicUtils.getCarId(topic);

		this.log.info("Received MQTT Message: \"" + topic + "\": \"" + message + "\"");

		if (this.isHeartbeat(topic))
		{
			this.log.info("Received Heartbeat from " + vehicleId);
			this.updateHeartbeat(vehicleId);
		}
		else if (this.isRegistration(topic))
		{
			this.log.info("Registered vehicle " + vehicleId + " with HeartbeatChecker.");
			this.addVehicle(vehicleId);
		}
		else if (this.isDeletion(topic))
		{
			this.log.info("Removing vehicle " + vehicleId + " from HeartbeatChecker.");
			this.removeVehicle(vehicleId);
		}
	}
}

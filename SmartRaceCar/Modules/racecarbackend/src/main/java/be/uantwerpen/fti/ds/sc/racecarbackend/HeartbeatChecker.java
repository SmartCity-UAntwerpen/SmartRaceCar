package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Lazy;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.lang.reflect.Type;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
@Component
class HeartbeatChecker implements MQTTListener
{
	private static final long CHECK_INTERVAL = 30000;		// Interval between heartbeat checks (in ms)
	private static final long MAX_DELTA = 90000;			// Maximum amount of time between consecutive heartbeats (in ms)

	private static class MQTTConstants
	{
		private static final Pattern HEARTBEAT_REGEX = Pattern.compile("racecar/[0-9]+/heartbeat");
	}

	private Logger log;
	private RESTUtils restUtils;
	private MQTTUtils mqttUtils;
	private VehicleManager vehicleManager;
	private Map<Long, Date> heartbeats;

	private boolean isHeartbeat(String topic)
	{
		Matcher matcher = HeartbeatChecker.MQTTConstants.HEARTBEAT_REGEX.matcher(topic);
		return matcher.matches();
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

	@Scheduled(fixedRate = CHECK_INTERVAL)
	private void checkBeats()
	{
		this.log.info("Heartbeats are being checked...");

		Date currentTime = new Date();

		for (long vehicleId : this.heartbeats.keySet())
		{
			final long delta = currentTime.getTime() - this.heartbeats.get(vehicleId).getTime();

			this.log.debug("Vehicle " + vehicleId + "'s last heartbeat came " + (delta / 1000) + "s ago.");

			if (delta > MAX_DELTA) //longer than 90 seconds
			{
				restUtils.getCall("delete/" + vehicleId);
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
	 * @param vehicleManager
	 */
	@Autowired
	public HeartbeatChecker(Parameters parameters, @Lazy VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(HeartbeatChecker.class);

		this.log.debug("Creating REST Utils for \"" + parameters.getRESTCarmanagerURL() + "\"...");

		this.restUtils = new RESTUtils(parameters.getRESTCarmanagerURL());
		this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(parameters.getMqttTopic());

		this.vehicleManager = vehicleManager;

		this.heartbeats = new HashMap<>();
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		long id = TopicUtils.getCarId(topic);

		if (this.vehicleManager.existsOld(id))
		{
			if (this.isHeartbeat(topic))
			{
				this.log.info("Received Heartbeat from " + id);
				this.updateHeartbeat(id);
			}
		}
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.MqttMessages;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Service
public class TopicParser
{
	private Logger log;
	private Configuration configuration;

	private static class MQTTConstants
	{
		private static final Pattern NORMAL_CAR_ID_REGEX = Pattern.compile("racecar/[A-Za-z_]+/([0-9]+)");
		private static final Pattern JOB_CAR_ID_REGEX = Pattern.compile("racecar/[A-Za-z_]+/([0-9]+)/[0-9]+");
	}

	public TopicParser(@Qualifier("topicParser") Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(TopicParser.class);
		this.configuration = configuration;
	}

	public boolean isMapChange(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
	}

	public boolean isHeartbeat(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.HEARTBEAT);
	}

	public boolean isRegistration(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.REGISTER);
	}

	public boolean isDeletion(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.DELETE);
	}

	public boolean isRouteUpdate(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE);
	}

	public boolean isProgressUpdate(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.PERCENTAGE);
	}

	public boolean isRegistrationComplete(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.REGISTRATION_DONE);
	}

	public boolean isLocationUpdate(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.LOCATION_UPDATE);
	}

	/**
	 * Extracts the car's id from any topic.
	 * If the topic doesn't match a racecar topic, -1 is returned
	 *
	 * @param topic
	 * @return
	 */
	public long getVehicleId(String topic)
	{
		Matcher matcher = MQTTConstants.NORMAL_CAR_ID_REGEX.matcher(topic);

		if (this.isRouteUpdate(topic) || this.isProgressUpdate(topic))
		{
			matcher = MQTTConstants.JOB_CAR_ID_REGEX.matcher(topic);
		}

		if (matcher.matches())
		{
			// Group 0 matches the entire string, so real capture groups start at index 1
			String idString = matcher.group(1);

			try
			{
				long id = Long.parseLong(idString);
				return id;
			}
			catch (NumberFormatException nfe)
			{
				this.log.error("Extracted invalid integer ('" + idString + "') from racecar topic ('" + topic + "').", nfe);
				return -1;
			}
		}
		else
		{
			log.warn("Failed to extract car id from topic: '" + topic + "'");
			return -1;
		}
	}
}

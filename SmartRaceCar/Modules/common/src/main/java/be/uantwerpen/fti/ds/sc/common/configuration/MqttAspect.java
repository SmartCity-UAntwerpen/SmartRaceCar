package be.uantwerpen.fti.ds.sc.common.configuration;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class MqttAspect extends Aspect
{
	private static final String PREFIX = "mqtt";

	// Set of keys relevant to this aspect
	private static final String MQTT_BROKER_KEY = PREFIX + ".broker";
	private static final String MQTT_USERNAME_KEY = PREFIX + ".username";
	private static final String MQTT_PASSWORD_KEY = PREFIX + ".password";
	private static final String MQTT_TOPIC_KEY = PREFIX + ".topic";

	// Set of default values for each key
	private static final String DEFAULT_MQTT_BROKER = "tcp://smartcity.ddns.net:1883";
	private static final String DEFAULT_MQTT_USERNAME = "root";
	private static final String DEFAULT_MQTT_PASSWORD = "smartcity";
	private static final String DEFAULT_MQTT_TOPIC = "/racecar";

	private String mqttBroker;
	private String mqttUsername;
	private String mqttPassword;
	private String mqttTopicPrefix;

	public MqttAspect (File configFile) throws IOException
	{
		super(AspectType.MQTT, PREFIX);

		try
		{
			FileInputStream inputStream = this.openFileStream(configFile);
			Properties properties = new Properties();
			properties.load(inputStream);

			this.mqttBroker = properties.getProperty(MQTT_BROKER_KEY, DEFAULT_MQTT_BROKER);
			this.mqttUsername = properties.getProperty(MQTT_USERNAME_KEY, DEFAULT_MQTT_USERNAME);
			this.mqttPassword = properties.getProperty(MQTT_PASSWORD_KEY, DEFAULT_MQTT_PASSWORD);
			this.mqttTopicPrefix = properties.getProperty(MQTT_TOPIC_KEY, DEFAULT_MQTT_TOPIC);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read MqttAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public MqttAspect(String mqttBroker, String mqttUsername, String mqttPassword, String mqttTopicPrefix)
	{
		super(AspectType.MQTT, PREFIX);
		this.mqttBroker = mqttBroker;
		this.mqttUsername = mqttUsername;
		this.mqttPassword = mqttPassword;
		this.mqttTopicPrefix = mqttTopicPrefix;
	}

	public String getMqttBroker()
	{
		return this.mqttBroker;
	}

	public String getMqttUsername()
	{
		return this.mqttUsername;
	}

	public String getMqttPassword()
	{
		return this.mqttPassword;
	}

	public String getMqttTopicPrefix()
	{
		return this.mqttTopicPrefix;
	}
}
package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
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
	private static final String DEFAULT_MQTT_TOPIC = "/racecar/";

	private Logger log;
	private String broker;
	private String username;
	private String password;
	private String topic;

	public MqttAspect (File configFile) throws IOException
	{
		super(AspectType.MQTT);
		this.log = LoggerFactory.getLogger(MqttAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.broker = properties.getProperty(MQTT_BROKER_KEY, DEFAULT_MQTT_BROKER);
			this.username = properties.getProperty(MQTT_USERNAME_KEY, DEFAULT_MQTT_USERNAME);
			this.password = properties.getProperty(MQTT_PASSWORD_KEY, DEFAULT_MQTT_PASSWORD);
			this.topic = properties.getProperty(MQTT_TOPIC_KEY, DEFAULT_MQTT_TOPIC);

			this.log.debug(MQTT_BROKER_KEY + " = " + this.broker);
			this.log.debug(MQTT_USERNAME_KEY + " = " + this.username);
			this.log.debug(MQTT_PASSWORD_KEY + " = " + this.password);
			this.log.debug(MQTT_TOPIC_KEY + " = " + this.topic);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read MqttAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public MqttAspect(String broker, String username, String password, String topic)
	{
		super(AspectType.MQTT);
		this.broker = broker;
		this.username = username;
		this.password = password;
		this.topic = topic;
	}

	public String getBroker()
	{
		return this.broker;
	}

	public String getUsername()
	{
		return this.username;
	}

	public String getPassword()
	{
		return this.password;
	}

	public String getTopic()
	{
		return this.topic;
	}
}

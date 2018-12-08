package be.uantwerpen.fti.ds.sc.smartracecar.common;

import org.springframework.beans.factory.annotation.Value;

public class Parameters
{
	@Value("${mqtt.broker}")
	private String mqttBroker;

	@Value("${mqtt.username}")
	private String mqttUserName;

	@Value("${mqtt.password}")
	private String mqttPassword;
	private String mqttTopic;

	private String restCarmanagerURL;

	public Parameters()
	{
		this(
				"tcp://smartcity.ddns.net:1883",
				"root",
				"smartcity",
				"http://smartcity.ddns.net:8081/carmanager", "racecar/#"
		);
	}

	public Parameters(String mqttBroker, String mqttUserName, String mqttPassword, String restCarmanagerURL)
	{
		this.mqttBroker = mqttBroker;
		this.mqttUserName = mqttUserName;
		this.mqttPassword = mqttPassword;
		this.mqttTopic = "racecar/#";

		this.restCarmanagerURL = restCarmanagerURL;
	}

	public Parameters(String mqttBroker, String mqttUserName, String mqttPassword, String restCarmanagerURL, String mqttTopic)
	{
		this.mqttBroker = mqttBroker;
		this.mqttUserName = mqttUserName;
		this.mqttPassword = mqttPassword;
		this.mqttTopic = mqttTopic;

		this.restCarmanagerURL = restCarmanagerURL;
	}

	public Parameters(Parameters parameters)
	{
		this.mqttBroker = parameters.getMqttBroker();
		this.mqttUserName = parameters.getMqttUserName();
		this.mqttPassword = parameters.getMqttPassword();
		this.mqttTopic = parameters.mqttTopic;
		this.restCarmanagerURL = parameters.getRESTCarmanagerURL();
	}

	public String getMqttBroker()
	{
		return this.mqttBroker;
	}

	public void setMqttBroker(String mqttBroker)
	{
		this.mqttBroker = mqttBroker;
	}

	public String getMqttUserName()
	{
		return this.mqttUserName;
	}

	public void setMqttUserName(String mqttUserName)
	{
		this.mqttUserName = mqttUserName;
	}

	public String getMqttPassword()
	{
		return this.mqttPassword;
	}

	public void setMqttPassword(String mqttPassword)
	{
		this.mqttPassword = mqttPassword;
	}

	public String getRESTCarmanagerURL()
	{
		return this.restCarmanagerURL;
	}

	public void setRestCarmanagerURL(String restCarmanagerURL)
	{
		this.restCarmanagerURL = restCarmanagerURL;
	}

	public String getMqttTopic()
	{
		return this.mqttTopic;
	}

	public void setMqttTopic(String mqttTopic)
	{
		this.mqttTopic = mqttTopic;
	}
}

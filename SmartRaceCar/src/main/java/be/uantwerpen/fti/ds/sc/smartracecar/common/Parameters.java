package be.uantwerpen.fti.ds.sc.smartracecar.common;

public class Parameters
{
	private String mqttBroker;
	private String mqttUserName;
	private String mqttPassword;
	private String mqttTopic;

	private String restURL;

	public Parameters()
	{
		this(
				"tcp://smartcity.ddns.net:1883",
				"root",
				"smartcity",
				"racecar/#",
				"http://smartcity.ddns.net:8081/carmanager"
		);
	}

	public Parameters(String mqttBroker, String mqttUserName, String mqttPassword, String mqttTopic, String restURL)
	{
		this.mqttBroker = mqttBroker;
		this.mqttUserName = mqttUserName;
		this.mqttPassword = mqttPassword;
		this.mqttTopic = mqttTopic;

		this.restURL = restURL;
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

	public String getRestURL()
	{
		return this.restURL;
	}

	public void setRestURL(String restURL)
	{
		this.restURL = restURL;
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

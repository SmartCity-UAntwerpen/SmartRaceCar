package be.uantwerpen.fti.ds.sc.common;


public class Parameters
{
	private String mqttBroker;

	private String mqttUserName;

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

	@Deprecated
	public Parameters(String mqttBroker, String mqttUserName, String mqttPassword, String restCarmanagerURL)
	{
		this.mqttBroker = mqttBroker;
		this.mqttUserName = mqttUserName;
		this.mqttPassword = mqttPassword;
		this.mqttTopic = "racecar/#";

		this.restCarmanagerURL = restCarmanagerURL;
	}

	@Deprecated
	public Parameters(String mqttBroker, String mqttUserName, String mqttPassword, String restCarmanagerURL, String mqttTopic)
	{
		this.mqttBroker = mqttBroker;
		this.mqttUserName = mqttUserName;
		this.mqttPassword = mqttPassword;
		this.mqttTopic = mqttTopic;

		this.restCarmanagerURL = restCarmanagerURL;
	}

	@Deprecated
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

	@Deprecated
	public void setMqttBroker(String mqttBroker)
	{
		this.mqttBroker = mqttBroker;
	}

	public String getMqttUserName()
	{
		return this.mqttUserName;
	}

	@Deprecated
	public void setMqttUserName(String mqttUserName)
	{
		this.mqttUserName = mqttUserName;
	}

	public String getMqttPassword()
	{
		return this.mqttPassword;
	}

	@Deprecated
	public void setMqttPassword(String mqttPassword)
	{
		this.mqttPassword = mqttPassword;
	}

	public String getRESTCarmanagerURL()
	{
		return this.restCarmanagerURL;
	}

	@Deprecated
	public void setRestCarmanagerURL(String restCarmanagerURL)
	{
		this.restCarmanagerURL = restCarmanagerURL;
	}

	public String getMqttTopic()
	{
		return this.mqttTopic;
	}
}

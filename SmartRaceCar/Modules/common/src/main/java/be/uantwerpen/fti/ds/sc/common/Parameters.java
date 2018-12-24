package be.uantwerpen.fti.ds.sc.common;

@Deprecated
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

	@Override
	public String toString()
	{
		StringBuilder builder = new StringBuilder();

		builder.append("MQTT BROKER: ");
		builder.append(this.mqttBroker);
		builder.append("\n");

		builder.append("MQTT USER NAME: ");
		builder.append(this.mqttUserName);
		builder.append("\n");

		builder.append("MQTT PASSWORD: ");
		builder.append(this.mqttPassword);
		builder.append("\n");

		builder.append("MQTT TOPIC: ");
		builder.append(this.mqttTopic);
		builder.append("\n");

		builder.append("REST CARMANAGER URL: ");
		builder.append(this.restCarmanagerURL);
		builder.append("\n");

		return builder.toString();
	}
}

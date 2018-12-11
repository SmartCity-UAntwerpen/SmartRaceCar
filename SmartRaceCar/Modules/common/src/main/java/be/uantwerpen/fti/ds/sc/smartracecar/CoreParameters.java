package be.uantwerpen.fti.ds.sc.smartracecar;

public class CoreParameters extends Parameters
{
	private boolean debug;

	public CoreParameters(boolean debug)
	{
		super();
		this.debug = debug;
	}

	public CoreParameters(String mqttBroker, String mqttUserName, String mqttPassword, String mqttTopic, String restURL, boolean debug)
	{
		super(mqttBroker, mqttUserName, mqttPassword, restURL, mqttTopic);
		this.debug = debug;
	}

	public boolean isDebug()
	{
		return this.debug;
	}

	public void setDebug(boolean debug)
	{
		this.debug = debug;
	}
}

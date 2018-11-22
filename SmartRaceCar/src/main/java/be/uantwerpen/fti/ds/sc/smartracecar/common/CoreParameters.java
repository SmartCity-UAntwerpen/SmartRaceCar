package be.uantwerpen.fti.ds.sc.smartracecar.common;

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
		super(mqttBroker, mqttUserName, mqttPassword, mqttTopic, restURL);
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

package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.Parameters;

public class CoreParameters extends Parameters
{
	private boolean debug;

	public CoreParameters()
	{
		super();
		this.debug = true;
	}

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

	public CoreParameters(Parameters parameters, boolean debug)
	{
		super(parameters);
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

package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.Parameters;

public class CoreParameters extends Parameters
{
	private boolean debug;
	private String navstackPath;

	public CoreParameters()
	{
		super();
		this.debug = true;
		this.navstackPath = "";
	}

	public CoreParameters(boolean debug, String navstackPath)
	{
		super();
		this.debug = debug;
		this.navstackPath = navstackPath;
	}

	public CoreParameters(String mqttBroker, String mqttUserName, String mqttPassword, String mqttTopic, String restURL, boolean debug, String navstackPath)
	{
		super(mqttBroker, mqttUserName, mqttPassword, restURL, mqttTopic);
		this.debug = debug;
		this.navstackPath = navstackPath;
	}

	public CoreParameters(Parameters parameters, boolean debug, String navstackPath)
	{
		super(parameters);
		this.debug = debug;
		this.navstackPath = navstackPath;
	}

	public boolean isDebug()
	{
		return this.debug;
	}

	public String getNavstackPath()
	{
		return navstackPath;
	}

	@Override
	public String toString()
	{
		StringBuilder builder = new StringBuilder();

		String parameters = super.toString();
		builder.append(parameters);

		builder.append("DEBUG WITHOUT ROS KERNEL: ");
		builder.append(this.debug);
		builder.append("\n");

		return builder.toString();
	}
}

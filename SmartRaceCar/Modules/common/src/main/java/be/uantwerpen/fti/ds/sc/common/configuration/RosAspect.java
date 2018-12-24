package be.uantwerpen.fti.ds.sc.common.configuration;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class RosAspect extends Aspect
{
	private static final String PREFIX = "Ros";

	private static final String ROS_DEBUG_MODE_KEY = PREFIX + ".debug";
	private static final String ROS_SERVER_URL_KEY = PREFIX + ".url";

	private static final String DEFAULT_ROS_DEBUG_MODE = "true";
	private static final String DEFAULT_ROS_SERVER_URL = "http://smartcity.ddns.net:8084";

	private boolean rosDebugMode;
	private String rosServerUrl;

	public RosAspect (File configFile) throws IOException
	{
		super(AspectType.ROS);
		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.rosDebugMode = Boolean.parseBoolean(properties.getProperty(ROS_DEBUG_MODE_KEY, DEFAULT_ROS_DEBUG_MODE));
			this.rosServerUrl = properties.getProperty(ROS_SERVER_URL_KEY, DEFAULT_ROS_SERVER_URL);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read RosAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public RosAspect (boolean rosDebugMode, String rosServerUrl)
	{
		super(AspectType.ROS);
		this.rosDebugMode = rosDebugMode;
		this.rosServerUrl = rosServerUrl;
	}

	public boolean isRosDebug()
	{
		return this.rosDebugMode;
	}

	public String getRosServerUrl()
	{
		return this.rosServerUrl;
	}
}

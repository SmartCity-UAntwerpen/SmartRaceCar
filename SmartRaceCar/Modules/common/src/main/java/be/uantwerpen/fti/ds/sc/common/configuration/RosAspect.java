package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class RosAspect extends Aspect
{
	private static final String PREFIX = "Ros";

	private static final String ROS_DEBUG_MODE_KEY = PREFIX + ".debug";
	private static final String ROS_SERVER_URL_KEY = PREFIX + ".url";
	private static final String ROS_SERVER_INCREASING_IDS = PREFIX + ".increasing_ids";
	private static final String[] KEYS = {ROS_DEBUG_MODE_KEY, ROS_SERVER_URL_KEY};

	private static final String DEFAULT_ROS_DEBUG_MODE = "true";
	private static final String DEFAULT_ROS_SERVER_URL = "http://smartcity.ddns.net:8084";
	private static final String DEFAULT_ROS_SERVER_INCREASING_IDS = "true";

	private Logger log;
	private boolean rosDebugMode;
	private boolean increasingIds;
	private String rosServerUrl;

	public RosAspect (File configFile) throws IOException
	{
		super(AspectType.ROS);
		this.log = LoggerFactory.getLogger(RosAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.rosDebugMode = Boolean.parseBoolean(properties.getProperty(ROS_DEBUG_MODE_KEY, DEFAULT_ROS_DEBUG_MODE));
			this.rosServerUrl = properties.getProperty(ROS_SERVER_URL_KEY, DEFAULT_ROS_SERVER_URL);
			this.increasingIds = Boolean.parseBoolean(properties.getProperty(ROS_SERVER_INCREASING_IDS, DEFAULT_ROS_SERVER_INCREASING_IDS));

			this.log.debug(ROS_DEBUG_MODE_KEY + " = " + this.rosDebugMode);
			this.log.debug(ROS_SERVER_URL_KEY + " = " + this.rosServerUrl);
			this.log.debug(ROS_SERVER_INCREASING_IDS + " = " + this.increasingIds);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public RosAspect (boolean rosDebugMode, String rosServerUrl, boolean increasingIds)
	{
		super(AspectType.ROS);
		this.log = LoggerFactory.getLogger(RosAspect.class);

		this.rosDebugMode = rosDebugMode;
		this.rosServerUrl = rosServerUrl;
		this.increasingIds = increasingIds;

		this.log.debug(ROS_DEBUG_MODE_KEY + " = " + this.rosDebugMode);
		this.log.debug(ROS_SERVER_URL_KEY + " = " + this.rosServerUrl);
		this.log.debug(ROS_SERVER_INCREASING_IDS + " = " + this.increasingIds);
	}

	public boolean isRosDebug()
	{
		return this.rosDebugMode;
	}

	public boolean isIncreasingIds()
	{
		return this.increasingIds;
	}

	public String getRosServerUrl()
	{
		return this.rosServerUrl;
	}
}

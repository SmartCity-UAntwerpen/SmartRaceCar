package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class NavStackAspect extends Aspect
{
	private static final String PREFIX = "navstack";

	// Set of keys relevant to this aspect
	private static final String NAVSTACK_PATH_KEY = PREFIX + ".path";
	private static final String[] KEYS = {NAVSTACK_PATH_KEY};

	// Set of default values for each key
	private static final String DEFAULT_NAVSTACK_PATH = "/home/ubuntu/Git/MAP2017/ROS/WS_Nav/src/f1tenth_2dnav/maps/";

	private Logger log;

	private String navStackPath;

	public NavStackAspect(File configFile) throws IOException
	{
		super(AspectType.NAVSTACK);

		this.log = LoggerFactory.getLogger(NavStackAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.navStackPath = properties.getProperty(NAVSTACK_PATH_KEY);

			this.log.debug(NAVSTACK_PATH_KEY + " = " + this.navStackPath);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public NavStackAspect(String navStackPath)
	{
		super(AspectType.NAVSTACK);
		this.navStackPath = navStackPath;
	}

	public String getNavStackPath()
	{
		return this.navStackPath;
	}
}

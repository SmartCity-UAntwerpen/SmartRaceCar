package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class CostAspect extends Aspect
{
	private static final String PREFIX = "Cost";

	private static final String INCREASING_IDS = PREFIX + ".increasing_ids";
	private static final String LOWER_RANGE = PREFIX + ".lower_range";
	private static final String UPPER_RANGE = PREFIX + ".upper_range";

	private static final String[] KEYS = {INCREASING_IDS, LOWER_RANGE, UPPER_RANGE};

	private static final String DEFAULT_ROS_SERVER_INCREASING_IDS = "true";
	private static final String DEFAULT_LOWER_RANGE = "0.0";
	private static final String DEFAULT_UPPER_RANGE = "50.0";

	private Logger log;
	private boolean increasingIds;
	private float lowerRange;
	private float upperRange;

	public CostAspect (File configFile) throws IOException
	{
		super(AspectType.COST);
		this.log = LoggerFactory.getLogger(RosAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.increasingIds = Boolean.parseBoolean(properties.getProperty(INCREASING_IDS, DEFAULT_ROS_SERVER_INCREASING_IDS));
			this.lowerRange = Float.parseFloat(properties.getProperty(LOWER_RANGE, DEFAULT_LOWER_RANGE));
			this.upperRange = Float.parseFloat(properties.getProperty(UPPER_RANGE, DEFAULT_UPPER_RANGE));

			this.log.debug(INCREASING_IDS + " = " + this.increasingIds);
			this.log.debug(LOWER_RANGE + " = " + this.lowerRange);
			this.log.debug(UPPER_RANGE + " = " + this.upperRange);

		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public boolean isIncreasingIds()
	{
		return this.increasingIds;
	}


	public float getLowerRange()
	{
		return this.lowerRange;
	}

	public float getUpperRange()
	{
		return this.upperRange;
	}
}

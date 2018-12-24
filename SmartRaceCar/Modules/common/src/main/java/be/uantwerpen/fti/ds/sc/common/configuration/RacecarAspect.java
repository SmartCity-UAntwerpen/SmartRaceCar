package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class RacecarAspect extends Aspect
{
	private static final String PREFIX = "Racecar";

	private static final String RACECAR_SERVER_URL_KEY = PREFIX + ".url";

	private static final String DEFAULT_RACECAR_SERVER_URL = "http://smartcity.ddns.net:8081/carmanager";

	private Logger log;
	private String racecarServerUrl;

	public RacecarAspect (File configFile) throws IOException
	{
		super(AspectType.RACECAR);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.racecarServerUrl = properties.getProperty(RACECAR_SERVER_URL_KEY, DEFAULT_RACECAR_SERVER_URL);

			this.log.debug(RACECAR_SERVER_URL_KEY + " = " + this.racecarServerUrl);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public RacecarAspect (String racecarServerUrl)
	{
		super(AspectType.RACECAR);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		this.racecarServerUrl = racecarServerUrl;
		this.log.debug(RACECAR_SERVER_URL_KEY + " = " + this.racecarServerUrl);
	}

	public String getRacecarServerUrl()
	{
		return this.racecarServerUrl;
	}
}

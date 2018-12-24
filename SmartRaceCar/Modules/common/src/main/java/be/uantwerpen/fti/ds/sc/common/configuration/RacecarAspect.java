package be.uantwerpen.fti.ds.sc.common.configuration;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class RacecarAspect extends Aspect
{
	private static final String PREFIX = "Racecar";

	private static final String RACECAR_SERVER_URL_KEY = PREFIX + ".url";

	private static final String DEFAULT_RACECAR_SERVER_URL = "http://smartcity.ddns.net:8081/carmanager";

	private String racecarServerUrl;

	public RacecarAspect (File configFile) throws IOException
	{
		super(AspectType.RACECAR);
		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.racecarServerUrl = properties.getProperty(RACECAR_SERVER_URL_KEY, DEFAULT_RACECAR_SERVER_URL);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read RacecarAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public RacecarAspect (String racecarServerUrl)
	{
		super(AspectType.RACECAR);
		this.racecarServerUrl = racecarServerUrl;
	}

	public String getRacecarServerUrl()
	{
		return this.racecarServerUrl;
	}
}

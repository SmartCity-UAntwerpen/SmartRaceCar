package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class BackboneAspect extends Aspect
{
	private static final String PREFIX = "Backbone";

	private static final String BACKBONE_DEBUG_MODE_KEY = PREFIX + ".debug";
	private static final String BACKBONE_SERVER_URL_KEY = PREFIX + ".url";

	private static final String DEFAULT_BACKBONE_DEBUG_MODE = "true";
	private static final String DEFAULT_BACKBONE_SERVER_URL = "http://smartcity.ddns.net:10000";

	private Logger log;
	private boolean backboneDebugMode;
	private String backboneServerUrl;

	public BackboneAspect (File configFile) throws IOException
	{
		super(AspectType.BACKBONE);
		this.log = LoggerFactory.getLogger(BackboneAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.backboneDebugMode = Boolean.parseBoolean(properties.getProperty(BACKBONE_DEBUG_MODE_KEY, DEFAULT_BACKBONE_DEBUG_MODE));
			this.backboneServerUrl = properties.getProperty(BACKBONE_SERVER_URL_KEY, DEFAULT_BACKBONE_SERVER_URL);

			this.log.debug(BACKBONE_DEBUG_MODE_KEY + " = " + this.backboneDebugMode);
			this.log.debug(BACKBONE_SERVER_URL_KEY + " = " + this.backboneServerUrl);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read RosAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public BackboneAspect (boolean backboneDebugMode, String backboneServerUrl)
	{
		super(AspectType.BACKBONE);
		this.log = LoggerFactory.getLogger(BackboneAspect.class);

		this.backboneDebugMode = backboneDebugMode;
		this.backboneServerUrl = backboneServerUrl;

		this.log.debug(BACKBONE_DEBUG_MODE_KEY + " = " + this.backboneDebugMode);
		this.log.debug(BACKBONE_SERVER_URL_KEY + " = " + this.backboneServerUrl);
	}

	public boolean isBackboneDebug()
	{
		return this.backboneDebugMode;
	}

	public String getBackboneServerUrl()
	{
		return this.backboneServerUrl;
	}
}

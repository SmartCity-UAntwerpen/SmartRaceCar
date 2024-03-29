package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class MapManagerAspect extends Aspect
{
	private static final String PREFIX = "Maps";

	private static final String MAP_PATH_KEY = PREFIX + ".path";
	private static final String CURRENT_MAP_KEY = PREFIX + ".current";
	private static final String DATABASE_DEBUG_KEY = PREFIX + ".database.debug";
	private static final String KEYS[] = {DATABASE_DEBUG_KEY, MAP_PATH_KEY, CURRENT_MAP_KEY};

	private static final String DEFAULT_DATABASE_DEBUG = "true";
	private static final String DEFAULT_MAP_PATH = "maps/";
	private static final String DEFAULT_CURRENT_MAP =  "U014Circle";

	private Logger log;
	private boolean databaseDebug;
	private String mapPath;
	private String currentMap;

	public MapManagerAspect (File configFile) throws IOException
	{
		super(AspectType.MAP_MANAGER);
		this.log = LoggerFactory.getLogger(MapManagerAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.databaseDebug = Boolean.parseBoolean(properties.getProperty(DATABASE_DEBUG_KEY, DEFAULT_DATABASE_DEBUG));
			this.mapPath = properties.getProperty(MAP_PATH_KEY, DEFAULT_MAP_PATH);
			this.currentMap = properties.getProperty(CURRENT_MAP_KEY, DEFAULT_CURRENT_MAP);

			this.log.debug(DATABASE_DEBUG_KEY + " = " + this.databaseDebug);
			this.log.debug(MAP_PATH_KEY + " = " + this.mapPath);
			this.log.debug(CURRENT_MAP_KEY + " = " + this.currentMap);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public MapManagerAspect (boolean databaseDebug, String mapPath, String currentMap)
	{
		super(AspectType.MAP_MANAGER);
		this.log = LoggerFactory.getLogger(MapManagerAspect.class);

		this.databaseDebug = databaseDebug;
		this.mapPath = mapPath;
		this.currentMap = currentMap;

		this.log.debug(DATABASE_DEBUG_KEY + " = " + this.databaseDebug);
		this.log.debug(MAP_PATH_KEY + " = " + this.mapPath);
		this.log.debug(CURRENT_MAP_KEY + " = " + this.currentMap);
	}

	public boolean isDatabaseDebug()
	{
		return this.databaseDebug;
	}

	public String getMapPath()
	{
		return this.mapPath;
	}

	public String getCurrentMap()
	{
		return this.currentMap;
	}
}

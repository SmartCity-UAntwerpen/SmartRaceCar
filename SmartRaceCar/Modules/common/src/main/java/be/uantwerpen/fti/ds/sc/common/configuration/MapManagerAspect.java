package be.uantwerpen.fti.ds.sc.common.configuration;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class MapManagerAspect extends Aspect
{
	private static final String PREFIX = "Maps";

	private static final String MAP_PATH_KEY = PREFIX + ".path";
	private static final String CURRENT_MAP_KEY = PREFIX + ".current";

	private static final String DEFAULT_MAP_PATH = "maps/";
	private static final String DEFAULT_CURRENT_MAP =  "U014Circle";

	private String mapPath;
	private String currentMap;

	public MapManagerAspect (File configFile) throws IOException
	{
		super(AspectType.MAP_MANAGER);
		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.mapPath = properties.getProperty(MAP_PATH_KEY, DEFAULT_MAP_PATH);
			this.currentMap = properties.getProperty(CURRENT_MAP_KEY, DEFAULT_CURRENT_MAP);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read MapManagerAspect from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public MapManagerAspect (String mapPath, String currentMap)
	{
		super(AspectType.MAP_MANAGER);
		this.mapPath = mapPath;
		this.currentMap = currentMap;
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

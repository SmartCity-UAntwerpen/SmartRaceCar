package be.uantwerpen.fti.ds.sc.smartracecar;

import org.springframework.beans.factory.annotation.Value;

public class MapManagerParameters extends BackendParameters
{
	@Value("${Maps.current}")
	private String currentMap;

	@Value("${Maps.path}")
	private String mapPath;

	public MapManagerParameters()
	{
		super();
		this.currentMap = "gangV";
		this.mapPath = "release/maps";
	}

	@Deprecated
	public MapManagerParameters(BackendParameters backendParameters, String currentMap, String mapPath)
	{
		super(backendParameters);
		this.currentMap = currentMap;
		this.mapPath = mapPath;
	}

	public String getCurrentMap()
	{
		return this.currentMap;
	}

	public String getMapPath()
	{
		return this.mapPath;
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

public class MapManagerParameters extends BackendParameters
{
	private String currentMap;
	private String mapPath;

	public MapManagerParameters()
	{
		super();
		this.currentMap = "U014";
		this.mapPath = "maps";
	}

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

	public void setCurrentMap(String map)
	{
		this.currentMap = map;
	}

	public String getMapPath()
	{
		return this.mapPath;
	}
}

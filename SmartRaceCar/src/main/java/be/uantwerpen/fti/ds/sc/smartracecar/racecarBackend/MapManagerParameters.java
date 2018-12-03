package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

public class MapManagerParameters extends BackendParameters
{
	private String currentMap;
	private String mapPath;

	public MapManagerParameters()
	{
		super();
		this.currentMap = "gangV";
		this.mapPath = "release/maps";
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

	public String getMapPath()
	{
		return this.mapPath;
	}
}

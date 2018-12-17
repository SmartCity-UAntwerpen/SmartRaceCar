package be.uantwerpen.fti.ds.sc.common;

/**
 * Model of a offline kept map.
 */
public class Map
{
	private String name; // Name of the mapfile
	private String path; // Path to the mapfile

	/**
	 * Model of a offline kept map.
	 *
	 * @param name The name of the map.
	 */
	public Map(String name, String path)
	{
		this.name = name;
		this.path = path;
	}

	/**
	 * Get the name of the map.
	 *
	 * @return The map name as String.
	 */
	public String getName()
	{
		return name;
	}

	public String getPath()
	{
		return path;
	}
}

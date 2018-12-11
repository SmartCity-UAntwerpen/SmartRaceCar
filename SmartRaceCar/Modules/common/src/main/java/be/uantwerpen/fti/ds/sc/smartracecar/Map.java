package be.uantwerpen.fti.ds.sc.smartracecar;

/**
 * Model of a offline kept map.
 */
public class Map
{

	private String name = ""; //Name of the mapfile.

	/**
	 * Model of a offline kept map.
	 *
	 * @param name The name of the map.
	 */
	public Map(String name)
	{
		this.name = name;
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
}

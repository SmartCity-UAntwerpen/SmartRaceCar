package be.uantwerpen.fti.ds.sc.smartracecar;

/**
 * Extended Model of the Super Point. This extended model also holds the ID of the waypoint.
 */
public class WayPoint extends Point
{

	private long id = 0; // Waypoint ID

	/**
	 * Extended Model of the Super Point. This extended model also holds the ID of the waypoint.
	 *
	 * @param id Waypoint ID.
	 * @param x  X offset from 0-point.
	 * @param y  Y offset from 0-point.
	 * @param z  Quaternion Z.
	 * @param w  Quaternion W.
	 */
	public WayPoint(long id, float x, float y, float z, float w)
	{
		super(x, y, z, w);
		this.id = id;
	}

	/**
	 * Get the ID of the waypoint.
	 *
	 * @return The ID of the waypoint.
	 */
	public long getID()
	{
		return id;
	}
}

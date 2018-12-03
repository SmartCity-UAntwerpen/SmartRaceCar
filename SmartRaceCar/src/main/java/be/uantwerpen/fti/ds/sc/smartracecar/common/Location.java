package be.uantwerpen.fti.ds.sc.smartracecar.common;

/**
 * Model that describes a location and the progress on a route. When start and end are the same
 * the vehicle is stationary. Then the percentage will be 100% as well.
 */
public class Location
{

	private long idVehicle; // ID of the vehicle
	private long idStart; // Waypoint ID of the starting location of the route.
	private long idEnd; // Waypoint ID of the end location of the route.
	private int percentage; // Percentage of the route completed between start and end.

	/**
	 * Model that describes a location and the progress on a route. When start and end are the same
	 * the vehicle is stationary. Then the percentage will be 100% as well.
	 *
	 * @param idVehicle  ID of the vehicle
	 * @param idStart    Waypoint ID of the starting location of the route.
	 * @param idEnd      Waypoint ID of the end location of the route.
	 * @param percentage Percentage of the route completed between start and end.
	 */
	public Location(long idVehicle, long idStart, long idEnd, int percentage)
	{
		this.idVehicle = idVehicle;
		this.idStart = idStart;
		this.idEnd = idEnd;
		this.percentage = percentage;
	}

	/**
	 * Set the start ID of the route.
	 *
	 * @param idStart Waypoint ID of the starting location of the route.
	 */
	public void setIdStart(long idStart)
	{
		this.idStart = idStart;
	}

	/**
	 * Set the end ID of the route.
	 *
	 * @param idEnd Waypoint ID of the ending location of the route.
	 */
	public void setIdEnd(long idEnd)
	{
		this.idEnd = idEnd;
	}

	/**
	 * Set the percentage completed of the route.
	 *
	 * @param percentage Percentage completed of the route.
	 */
	public void setPercentage(int percentage)
	{
		this.percentage = percentage;
	}

	/**
	 * Get the end ID of the route.
	 *
	 * @return Waypoint ID of the ending location of the route.
	 */
	public long getIdEnd()
	{
		return idEnd;
	}

	/**
	 * Get the percentage completed of the route.
	 *
	 * @return Percentage completed of the route.
	 */
	public int getPercentage()
	{
		return percentage;
	}

}

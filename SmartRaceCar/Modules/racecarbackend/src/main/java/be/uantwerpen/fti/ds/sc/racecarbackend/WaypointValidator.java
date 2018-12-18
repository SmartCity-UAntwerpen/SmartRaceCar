package be.uantwerpen.fti.ds.sc.racecarbackend;

public interface WaypointValidator
{
	/**
	 * Checks if a waypoint exists or not.
	 *
	 * @param waypointId    The ID of the waypoint to be checked.
	 * @return              True if the waypoint exists, false if it does not.
	 */
	public boolean exists(long waypointId);
}

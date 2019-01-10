package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Point;

public interface CoordinateRepository
{
	/**
	 * Get the coordinates (x,y,z,w) of a point with a certain ID.
	 * @param waypointId
	 * @return
	 */
	public Point getCoordinates(long waypointId);
}

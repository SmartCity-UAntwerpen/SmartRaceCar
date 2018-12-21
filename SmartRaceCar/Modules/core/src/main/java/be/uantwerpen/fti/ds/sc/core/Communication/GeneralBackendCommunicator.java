package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.HashMap;

public interface GeneralBackendCommunicator
{
	/**
	 * Will register the car on the backend, starting from a certain startPoint (waypoint)
	 * @param startPoint
	 * @return ID of the car
	 */
	public long register(long startPoint);


	public void disconnect(long ID);
}

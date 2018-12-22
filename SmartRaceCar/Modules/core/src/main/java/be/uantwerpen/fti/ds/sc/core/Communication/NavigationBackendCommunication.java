package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.HashMap;

public interface NavigationBackendCommunication
{
	/**
	 * Will request the waypoints of the current map from the backend
	 * @return
	 */
	public HashMap<Long, WayPoint> requestWayPoints();
}

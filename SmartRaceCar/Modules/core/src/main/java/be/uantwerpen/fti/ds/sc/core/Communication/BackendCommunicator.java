package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.HashMap;

public interface BackendCommunicator
{
	public long register(long startPoint);
	public HashMap<Long, WayPoint> requestWayPoints();
	public void disconnect(long ID);
	public RESTUtils getRESTUtils(); // TODO remove
}

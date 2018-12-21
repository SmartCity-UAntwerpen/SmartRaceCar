package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.List;

public interface NavigationVehicleCommunication
{
	public void timeRequest(List<Point> points);
	public void sendWheelStates(float throttle, float steer);
	public void sendCurrentPosition(WayPoint wayPoint);
	public void sendStartpoint(WayPoint startPoint);
	public void sendNextWayPoint(WayPoint wayPoint);
}

package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.List;

public interface CoreLinkerCommunicator
{
	public void start();
	public void connect();
	public void disconnect();
	public void sendStartpoint(WayPoint startPoint);
	public void sendCurrentPosition(WayPoint wayPoint);
	public void timeRequest(List<Point> points);
	public TCPUtils getTCPUtils(); // TODO remove
}

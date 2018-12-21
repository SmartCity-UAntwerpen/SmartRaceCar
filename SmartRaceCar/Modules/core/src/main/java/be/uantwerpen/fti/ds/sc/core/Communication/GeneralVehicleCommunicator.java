package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import be.uantwerpen.fti.ds.sc.common.WayPoint;

import java.util.List;

public interface GeneralVehicleCommunicator
{
	public void start();
	public void connect();
	public void disconnect();
	public TCPUtils getTCPUtils(); // TODO remove
}

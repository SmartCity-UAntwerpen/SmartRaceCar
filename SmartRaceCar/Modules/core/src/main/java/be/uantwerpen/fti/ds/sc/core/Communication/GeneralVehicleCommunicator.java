package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.TCPUtils;

public interface GeneralVehicleCommunicator
{
	public void start();
	public void connect();
	public void disconnect();
}

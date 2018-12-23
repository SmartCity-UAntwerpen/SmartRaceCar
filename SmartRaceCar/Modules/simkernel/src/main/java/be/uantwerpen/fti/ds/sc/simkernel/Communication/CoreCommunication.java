package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.Cost;

public interface CoreCommunication
{
	public void start();
	public void connect();
	public void wayPointReached();
	public void percentageUpdate(Cost cost);
	public void sendCost(Cost cost);	 		// TODO check if necessary
	public void sendTiming(Cost cost);
	public void disconnect();
	public void exit();
}

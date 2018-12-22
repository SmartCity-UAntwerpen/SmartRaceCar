package be.uantwerpen.fti.ds.sc.simkernel.Communication;

public interface CoreCommunication
{
	public void start();
	public void connectReceive();
	public void wayPointReached();
	public void percentageUpdate();
	public void sendCost();	 		// TODO check if necessary
	public void sendTiming();
}

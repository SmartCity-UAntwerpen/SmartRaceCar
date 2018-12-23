package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;

public class CoreCommunicator implements TCPListener, CoreCommunication
{
	private Logger log;
	private TCPUtils tcpUtils;
	private MessageListener listener;

	public CoreCommunicator(int serverport, int clientport, MessageListener listener)
	{
		this.log = LoggerFactory.getLogger(CoreCommunicator.class);
		this.tcpUtils = new TCPUtils(clientport, serverport, this);
		this.log.info("Startup parameters: TCP Server Port:" + serverport + " | TCP Client Port:" + clientport);

		this.listener = listener;
	}

	@Override
	public String parseTCP(String message) throws IOException
	{
		this.listener.notify(message);
		return null;
	}

	@Override
	public void start()
	{
		this.tcpUtils.start();
	}

	@Override
	public void connect()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString(Messages.SIMKERNEL.CONNECT));
	}

	@Override
	public void wayPointReached()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString(Messages.SIMKERNEL.ARRIVED_WAYPOINT));
		this.log.info("Arrived at waypoint. Waiting for next order.");
	}

	@Override
	public void percentageUpdate(Cost cost)
	{
		for (int i = 0; i <= 20; i++)
		{
			try
			{
				Thread.sleep((cost.getWeight() * 1000) / 20);
				Location location = new Location(0, 0, 0, i * 5);
				this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(Messages.SIMKERNEL.PERCENTAGE, location));
				this.log.info("travelled " + i * 5 + "% of total route.");
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}

	@Override
	public void sendTiming(Cost cost)
	{
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(Messages.SIMKERNEL.COST_TIMING, cost));
	}

	@Override
	public void disconnect()
	{
		this.tcpUtils.closeTCP();
	}

	@Override
	public void exit()
	{
		this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString(Messages.SIMKERNEL.EXIT));
	}
}

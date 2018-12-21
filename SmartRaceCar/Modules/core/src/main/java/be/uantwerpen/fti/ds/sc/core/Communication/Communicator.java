package be.uantwerpen.fti.ds.sc.core.Communication;
import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.core.CoreParameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class Communicator implements CoreLinkerCommunicator
{
	private CoreParameters params;
	private Logger log;
	private TCPUtils tcpUtils;
	private RESTUtils restUtils;

	public Communicator(CoreParameters params, TCPListener listener, int clientPort, int serverPort)
	{
		this.log = LoggerFactory.getLogger(Communicator.class);
		this.params = params;
		this.tcpUtils = new TCPUtils(clientPort, serverPort, listener);
		this.restUtils = new RESTUtils(this.params.getRESTCarmanagerURL());
	}

	public void start()
	{
		this.tcpUtils.start();
	}

	public void timeRequest(List<Point> points)
	{
		this.log.debug("Performing timing request");
		this.tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("costtiming", points));
	}

	public void connect()
	{
		this.log.info("Trying to connect to car");
		if(!this.params.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
		}
		else
		{
			this.log.info("Debug mode -> not connecting to car");
		}
	}

	public void sendStartpoint(WayPoint startPoint)
	{
		this.log.debug("Sending start point");
		if(!this.params.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startpoint", startPoint));
		}
		else
		{
			this.log.info("Debug mode -> not sending start point");
		}
	}

	public void sendCurrentPosition(WayPoint wayPoint)
	{
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("currentPosition", wayPoint));
	}

	public void disconnect()
	{
		if(!this.params.isDebug())
		{
			this.tcpUtils.closeTCP();
		}
		else
		{
			this.log.info("Debug mode -> not closing TCP");
		}
	}

	public TCPUtils getTCPUtils()
	{
		return this.tcpUtils;
	}
}

package be.uantwerpen.fti.ds.sc.core.Communication;
import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.KernelAspect;
import be.uantwerpen.fti.ds.sc.core.Drive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class VehicleCommunicator implements GeneralVehicleCommunicator, MapVehicleCommunicator, NavigationVehicleCommunication
{
	private Configuration configuration;
	private Logger log;
	private TCPUtils tcpUtils;

	public VehicleCommunicator(Configuration configuration, TCPListener listener, int clientPort, int serverPort)
	{
		this.log = LoggerFactory.getLogger(VehicleCommunicator.class);
		this.configuration = configuration;
		this.tcpUtils = new TCPUtils(clientPort, serverPort, listener);
	}

	@Override
	public void start()
	{
		this.tcpUtils.start();
	}

	@Override
	public void timeRequest(List<Point> points)
	{
		this.log.debug("Performing timing request");
		this.tcpUtils.sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord(TcpMessages.Core.COST_TIMING, points));
	}

	@Override
	public void connect()
	{
		KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
		if(!kernelAspect.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.keywordToJSONString(TcpMessages.Core.CONNECT));
		}
		else
		{
			this.log.info("Debug mode -> not connecting to car");
		}
	}

	@Override
	public void sendStartpoint(WayPoint startPoint)
	{
		this.log.debug("Sending start point");
		KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
		if(!kernelAspect.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(TcpMessages.Core.START_POINT, startPoint));
		}
		else
		{
			this.log.info("Debug mode -> not sending start point");
		}
	}

	@Override
	public void sendCurrentPosition(WayPoint wayPoint)
	{
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(TcpMessages.Core.CURRENT_POSITION, wayPoint));
	}

	@Override
	public void disconnect()
	{
		KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
		if(!kernelAspect.isDebug())
		{
			this.tcpUtils.closeTCP();
		}
		else
		{
			this.log.info("Debug mode -> not closing TCP");
		}
	}

	@Override
	public void setMap(Map map)
	{
		String json = JSONUtils.objectToJSONStringWithKeyWord(TcpMessages.Core.CURRENT_MAP, map);
		this.log.info("Setting current map on NAVSTACK to " + json);
		this.tcpUtils.sendUpdate(json);
	}

	@Override
	public void sendWheelStates(float throttle, float steer)
	{
		this.log.info("Sending wheel state Throttle:" + throttle + ", Steer:" + steer + ".");
		KernelAspect kernelAspect = (KernelAspect) this.configuration.get(AspectType.KERNEL);
		if(!kernelAspect.isDebug())
		{
			this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(TcpMessages.Core.DRIVE, new Drive(steer, throttle)));
		}
		else
		{
			this.log.info("Debug mode -> not sending wheel states");
		}
	}

	@Override
	public void sendNextWayPoint(WayPoint wayPoint)
	{
		this.tcpUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord(TcpMessages.Core.NEXT_WAYPOINT, wayPoint));
	}
}

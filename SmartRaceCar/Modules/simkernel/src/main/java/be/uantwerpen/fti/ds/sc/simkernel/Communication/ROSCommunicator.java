package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.Cost;
import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.RosAspect;
import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.List;

public class ROSCommunicator implements ROSCommunication
{
	private RESTUtils restUtils;

	public ROSCommunicator(Configuration configuration)
	{
		RosAspect aspect = (RosAspect) configuration.get(AspectType.ROS);
		this.restUtils = new RESTUtils((aspect.getRosServerUrl()));
	}

	@Override
	public Cost requestCost(List<Point> points, Type typeOfCost) throws IOException
	{
		return (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.post("calcWeight", JSONUtils.arrayToJSONString(points), MediaType.APPLICATION_JSON_TYPE), typeOfCost);
	}
}

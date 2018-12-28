package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.Cost;
import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.RosAspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.List;

public class ROSCommunicator implements ROSCommunication
{
	private Logger log;
	private RESTUtils restUtils;

	public ROSCommunicator(Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(ROSCommunicator.class);

		RosAspect aspect = (RosAspect) configuration.get(AspectType.ROS);
		this.restUtils = new RESTUtils((aspect.getRosServerUrl()));
	}

	@Override
	public Cost requestCost(List<Point> points, Type typeOfCost) throws IOException
	{
		String json =  JSONUtils.arrayToJSONString(points);

		this.log.debug("JSON of weight request = " + json);

		return (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.post("calcWeight", json, MediaType.APPLICATION_JSON_TYPE), typeOfCost);
	}
}

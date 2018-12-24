package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.Cost;
import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.simkernel.SimkernelParameters;

import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.List;

public class ROSCommunicator implements ROSCommunication
{
	private SimkernelParameters params;
	private RESTUtils restUtils;

	public ROSCommunicator(SimkernelParameters params)
	{
		this.params = params;
		this.restUtils = new RESTUtils((this.params.getROSServerURL()));
	}

	@Override
	public Cost requestCost(List<Point> points, Type typeOfCost) throws IOException
	{
		return (Cost) JSONUtils.getObjectWithKeyWord(this.restUtils.post("calcWeight", JSONUtils.arrayToJSONString(points), MediaType.APPLICATION_JSON_TYPE), typeOfCost);
	}
}

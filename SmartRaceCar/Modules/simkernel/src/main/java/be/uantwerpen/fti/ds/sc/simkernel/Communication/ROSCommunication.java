package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.Cost;
import be.uantwerpen.fti.ds.sc.common.Point;

import java.io.IOException;
import java.lang.reflect.Type;
import java.util.List;

public interface ROSCommunication
{
	public Cost requestCost(List<Point> points, Type typeOfCost) throws IOException;
}

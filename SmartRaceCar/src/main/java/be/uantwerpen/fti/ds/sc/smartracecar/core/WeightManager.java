package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Cost;
import be.uantwerpen.fti.ds.sc.smartracecar.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

import java.util.ArrayList;
import java.util.List;

public class WeightManager
{
	private Core core;


	public WeightManager(Core core)
	{
		this.core = core;
	}

	/**
	 * Called by incoming cost calculation requests. Sends the request further to the RosKernel/SimKernel.
	 *
	 * @param wayPointIDs Array of waypoint ID's to have their cost calculated.
	 */
	public void costRequest(long[] wayPointIDs)
	{
		List<Point> points = new ArrayList<>();
		points.add(this.core.getWayPoints().get(wayPointIDs[0]));
		points.add(this.core.getWayPoints().get(wayPointIDs[1]));
		if (!this.core.getParams().isDebug())
		{
			this.core.getTcpUtils().sendUpdate(JSONUtils.arrayToJSONStringWithKeyWord("cost", points));
		}
		else
		{
			costComplete(new Cost(false, 5, 5, this.core.getID()));
		}
		Log.logInfo("CORE", "Cost request received between waypoints " + wayPointIDs[0] + " and " + wayPointIDs[1] + ". Calculating.");
	}

	/**
	 * Called by received response from SimKernel/RosKernel of cost calculation request.
	 *
	 * @param cost Cost object containing the calculated weights.
	 */
	public void costComplete(Cost cost)
	{
		Log.logInfo("CORE", "Cost request calculated.");
		cost.setStatus(this.core.isOccupied());
		cost.setIdVehicle(this.core.getID());
		this.core.getMqttUtils().publishMessage("racecar/" + this.core.getID() + "/costanswer", JSONUtils.objectToJSONString(cost));
	}
}

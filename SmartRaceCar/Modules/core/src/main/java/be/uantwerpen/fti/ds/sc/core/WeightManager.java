package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class WeightManager implements MQTTListener
{
	private Core core;
	private Logger log;
	private MQTTUtils mqttUtils;


	public WeightManager(Core core)
	{
		this.log = LoggerFactory.getLogger(WeightManager.class);
		this.core = core;
		this.mqttUtils = new MQTTUtils(this.core.getParams().getMqttBroker(), this.core.getParams().getMqttUserName(), this.core.getParams().getMqttPassword(), this);
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
			costCalculationComplete(new Cost(false, 5, 5, this.core.getID()));
		}
		this.log.info("Cost request received between waypoints " + wayPointIDs[0] + " and " + wayPointIDs[1] + ". Calculating.");
	}

	/**
	 * Called by received response from SimKernel/RosKernel of cost calculation request.
	 *
	 * @param cost Cost object containing the calculated weights.
	 */
	public void costCalculationComplete(Cost cost)
	{
		this.log.info("Cost request calculated.");
		cost.setStatus(this.core.isOccupied());
		cost.setIdVehicle(this.core.getID());
		this.mqttUtils.publishMessage("racecar/" + this.core.getID() + "/costanswer", JSONUtils.objectToJSONString(cost));
	}


	@Override
	public void parseMQTT(String topic, String message)
	{

	}
}

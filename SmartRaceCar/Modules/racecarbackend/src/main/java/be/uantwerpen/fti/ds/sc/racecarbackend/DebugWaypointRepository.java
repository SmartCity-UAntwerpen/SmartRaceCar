package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.WayPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.util.*;

@Service
public class DebugWaypointRepository
{
	private Logger log;

	public DebugWaypointRepository()
	{
		this.log = LoggerFactory.getLogger(DebugWaypointRepository.class);
	}

	/**
	 * Request all possible wayPoints from the BackBone through a REST Get request.
	 */
	public Map<Long, WayPoint> loadWayPoints(String mapName)
	{
		this.log.info("Loading wayPoints for " + mapName);

		Map<Long, WayPoint> waypoints = new HashMap<>();


		// Temp wayPoints for when they can't be requested from back-end services.
		switch (mapName)
		{
			case "zbuilding":
				waypoints.put(46L, new WayPoint(46, 0.5f, 0.0f, -1.0f, 0.02f));
				waypoints.put(47L, new WayPoint(47, -13.4f, -0.53f, 0.71f, 0.71f));
				waypoints.put(48L, new WayPoint(48, -27.14f, -1.11f, -0.3f, 0.95f));
				waypoints.put(49L, new WayPoint(49, -28.25f, -9.19f, -0.71f, 0.71f));
				break;
			case "V314":
				waypoints.put(46L, new WayPoint(46, -3.0f, -1.5f, 0.07f, 1.00f));
				waypoints.put(47L, new WayPoint(47, 1.10f, -1.20f, 0.07f, 1.00f));
				waypoints.put(48L, new WayPoint(48, 4.0f, -0.90f, -0.68f, 0.73f));
				waypoints.put(49L, new WayPoint(49, 4.54f, -4.49f, -0.60f, 0.80f));
				break;
			case "gangV":
				waypoints.put(46L, new WayPoint(46, -6.1f, -28.78f, 0.73f, 0.69f));
				waypoints.put(47L, new WayPoint(47, -6.47f, -21.69f, 0.66f, 0.75f));
				waypoints.put(48L, new WayPoint(48, -5.91f, -1.03f, 0.52f, 0.85f));
				waypoints.put(49L, new WayPoint(49, 6.09f, 0.21f, -0.04f, 1.00f));
				break;
			case "U014":
				waypoints.put(0L, new WayPoint(0, 0.01f, -0.01f, 0.01f, 0.99f));
				waypoints.put(1L, new WayPoint(1, 4.57f, 2.92f, 0.75f, 0.67f));
				waypoints.put(2L, new WayPoint(2, 4.47f, 6.02f, 0.74f, 0.67f));
				waypoints.put(3L, new WayPoint(3, 4.20f, 10.01f, 0.74f, 0.67f));
				waypoints.put(4L, new WayPoint(4, 3.97f, 13.04f, 0.75f, 0.66f));
				break;
			default:
				log.warn("There are no default wayPoints for \"" + mapName + "\".");
				waypoints.put(46L, new WayPoint(46, 0.5f, 0.0f, -1.0f, 0.02f));
				waypoints.put(47L, new WayPoint(47, -13.4f, -0.53f, 0.71f, 0.71f));
				waypoints.put(48L, new WayPoint(48, -27.14f, -1.11f, -0.3f, 0.95f));
				waypoints.put(49L, new WayPoint(49, -28.25f, -9.19f, -0.71f, 0.71f));
		}

		this.log.info("All possible wayPoints(" + waypoints.size() + ") received.");

		return waypoints;
	}
}
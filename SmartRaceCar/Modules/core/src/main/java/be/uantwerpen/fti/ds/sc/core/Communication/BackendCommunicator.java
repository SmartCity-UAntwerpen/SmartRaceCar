package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Messages;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.common.WayPoint;
import be.uantwerpen.fti.ds.sc.core.CoreParameters;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.ws.rs.core.MediaType;
import java.lang.reflect.Type;
import java.util.HashMap;

public class BackendCommunicator implements GeneralBackendCommunicator, MapBackendCommunicator, NavigationBackendCommunication
{
	private Logger log;
	private CoreParameters params;
	private RESTUtils restUtils;

	public BackendCommunicator(CoreParameters params)
	{
		this.log = LoggerFactory.getLogger(BackendCommunicator.class);
		this.params = params;
		this.restUtils = new RESTUtils(this.params.getRESTCarmanagerURL());
	}


	@Override
	public long register(long startPoint)
	{
		String id = this.restUtils.get(Messages.BACKEND.REGISTER + "/" + Long.toString(startPoint), MediaType.TEXT_PLAIN_TYPE);
		long ID = Long.parseLong(id, 10);
		this.log.info("Vehicle received ID " + ID + ".");
		return ID;
	}

	@Override
	public HashMap<Long, WayPoint> requestWayPoints()
	{
		this.log.info("Requesting waypoints");
		Type typeOfHashMap = new TypeToken<HashMap<Long, WayPoint>>()
		{
		}.getType();
		HashMap<Long, WayPoint> wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.get(Messages.BACKEND.GET_WAYPOINTS, MediaType.APPLICATION_JSON_TYPE), typeOfHashMap);

		for (WayPoint wayPoint : wayPoints.values())
		{
			this.log.info("Waypoint " + wayPoint.getID() + " added: " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW());
		}

		this.log.info("All possible waypoints(" + wayPoints.size() + ") received.");

		return wayPoints;
	}

	@Override
	public void disconnect(long ID)
	{
		this.restUtils.get(Messages.BACKEND.DELETE + "/" + ID);
	}

	@Override
	public String getMapName()
	{
		return this.restUtils.get(Messages.BACKEND.GET_MAP_NAME, MediaType.TEXT_PLAIN_TYPE);
	}

	@Override
	public void downloadMap(String mapName)
	{
		this.log.info("Current used map '" + mapName + "' not found. Downloading... to " + this.params.getNavstackPath() + "/" + mapName);
		this.restUtils.getFile( Messages.BACKEND.GET_MAP_PGM + "/" + mapName, this.params.getNavstackPath(), mapName, "pgm");
		this.restUtils.getFile( Messages.BACKEND.GET_MAP_YAML + "/" + mapName, this.params.getNavstackPath(), mapName, "yaml");
	}
}

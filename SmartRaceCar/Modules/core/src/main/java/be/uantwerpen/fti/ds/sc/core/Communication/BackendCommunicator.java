package be.uantwerpen.fti.ds.sc.core.Communication;

import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.RESTMessages;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.common.WayPoint;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.NavStackAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.RacecarAspect;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.HashMap;

public class BackendCommunicator implements GeneralBackendCommunicator, MapBackendCommunicator, NavigationBackendCommunication
{
	private Logger log;
	private Configuration configuration;
	private RESTUtils restUtils;

	public BackendCommunicator(Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(BackendCommunicator.class);
		this.configuration = configuration;

		RacecarAspect racecarAspect = (RacecarAspect) this.configuration.get(AspectType.RACECAR);
		this.restUtils = new RESTUtils(racecarAspect.getRacecarServerUrl());
	}


	@Override
	public long register(long startPoint)
	{
		String id = this.restUtils.get(RESTMessages.Backend.REGISTER + "/" + Long.toString(startPoint), MediaType.TEXT_PLAIN_TYPE);
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
		HashMap<Long, WayPoint> wayPoints = (HashMap<Long, WayPoint>) JSONUtils.getObjectWithKeyWord(restUtils.get(RESTMessages.Backend.GET_WAYPOINTS, MediaType.APPLICATION_JSON_TYPE), typeOfHashMap);

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
		this.restUtils.delete(RESTMessages.Backend.DELETE + "/" + ID);
	}

	@Override
	public String getMapName()
	{
		return this.restUtils.get(RESTMessages.Backend.GET_MAP_NAME, MediaType.TEXT_PLAIN_TYPE);
	}

	@Override
	public void downloadMap(String mapName)
	{
		NavStackAspect navStackAspect = (NavStackAspect) this.configuration.get(AspectType.NAVSTACK);

		this.log.info("Current used map '" + mapName + "' not found. Downloading... to " + navStackAspect.getNavStackPath() + "/" + mapName);

		try
		{
			this.restUtils.getFile(RESTMessages.Backend.GET_MAP_PGM + "/" + mapName, navStackAspect.getNavStackPath(), mapName, "pgm");
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to download map PGM file.", ioe);
		}

		try
		{
			this.restUtils.getFile(RESTMessages.Backend.GET_MAP_YAML + "/" + mapName, navStackAspect.getNavStackPath(), mapName, "yaml");
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to download map YAML file.", ioe);
		}
	}
}

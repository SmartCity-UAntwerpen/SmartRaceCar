package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import javax.servlet.http.HttpServletResponse;
import javax.swing.text.html.HTMLDocument;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;
import java.io.File;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class MapManager implements MQTTListener
{
	private LogbackWrapper log;
	private MapManagerParameters params;

	private VehicleManager vehicleManager;
	private MQTTUtils mqttUtils;
	private RESTUtils backboneRESTUtils;

	private Map<Long, WayPoint> wayPoints;
	private String currentMap;
	private String mapPath;

	public MapManager(MapManagerParameters params, VehicleManager vehicleManager)
	{
		this.log = new LogbackWrapper();
		this.params = params;

		this.currentMap = params.getCurrentMap();
		this.mapPath = params.getMapPath();

		this.mqttUtils = new MQTTUtils(params.getMqttBroker(), params.getMqttUserName(), params.getMqttPassword(), this);
		this.backboneRESTUtils = new RESTUtils(params.getBackboneRESTURL());

		this.vehicleManager = vehicleManager;

		this.wayPoints = new HashMap<>();
		this.loadWayPoints();
	}

	public boolean exists(long id)
	{
		return wayPoints.containsKey(id);
	}

	public void setVehicleManager(VehicleManager vehicleManager)
	{
		this.vehicleManager = vehicleManager;
	}

	/**
	 * REST GET server service to get the currently used map.
	 *
	 * @return REST response of the type Text Plain containing the mapname.
	 */
	@GET
	@Path("getmapname")
	@Produces("text/plain")
	public String getMapName()
	{
		return this.currentMap;
	}

	/**
	 * REST GET server service to download a map's PGM file by name.
	 *
	 * @param mapname the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@GET
	@Path("getmappgm/{mapname}")
	@Produces("application/octet-stream")
	public Response getMapPGM(@PathParam("mapname") final String mapname, @Context HttpServletResponse response) throws UnsupportedEncodingException
	{
		StreamingOutput fileStream = output ->
		{
			try
			{
				java.nio.file.Path path = Paths.get(this.mapPath + "/" + mapname + ".pgm");
				byte[] data = Files.readAllBytes(path);
				output.write(data);
				output.flush();
			} catch (Exception e)
			{
				System.out.println("error " + e);
				response.sendError(HttpServletResponse.SC_NOT_FOUND, mapname + ".pgm not found");
			}
		};
		return Response
				.ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
				.header("content-disposition", "attachment; filename = " + mapname + ".pgm")
				.build();

	}

	/**
	 * REST GET server service to get all currently used wayPoints by F1 vehicles.
	 *
	 * @return REST response of the type JSON containing all wayPoints.
	 */
	@GET
	@Path("getwaypoints")
	@Produces("application/json")
	public String getWayPoints()
	{
		return JSONUtils.objectToJSONStringWithKeyWord("wayPoints", this.wayPoints);
	}

	/**
	 * REST GET server service to download a map's YAML file by name.
	 *
	 * @param mapname the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@GET
	@Path("getmapyaml/{mapname}")
	@Produces("application/octet-stream")
	public Response getMapYAML(@PathParam("mapname") final String mapname, @Context HttpServletResponse response)
	{
		StreamingOutput fileStream = output ->
		{
			try
			{
				java.nio.file.Path path = Paths.get(mapPath + "/" + mapname + ".yaml");
				byte[] data = Files.readAllBytes(path);
				output.write(data);
				output.flush();
			} catch (Exception e)
			{
				response.sendError(HttpServletResponse.SC_NOT_FOUND, mapname + ".yaml not found");
			}
		};
		return Response
				.ok(fileStream, MediaType.APPLICATION_OCTET_STREAM)
				.header("content-disposition", "attachment; filename = " + mapname + ".yaml")
				.build();
	}

	/**
	 * Rest command that can be called to change the map used by the racecars at runtime
	 *
	 * @param mapName name of the new map
	 * @return
	 */
	@GET
	@Path("changeMap/{mapName}")
	@Produces("text/plain")
	public String changeMap(@PathParam("mapName") String mapName)
	{
		File f = new File(mapPath + "/" + mapName + ".yaml");
		if (f.exists() && !f.isDirectory())
		{
			this.currentMap = mapName;

			for (Iterator it = this.vehicleManager.getIdIterator(); it.hasNext(); )
			{
				int ID = (int) it.next();

				this.log.info("RACECAR_BACKEND", "change map command send to vehicle with ID: " + ID);
				this.mqttUtils.publishMessage("racecar/" + ID + "/changeMap", mapName);
				loadWayPoints();
			}
			return "Command was executed to change map";
		} else
		{
			this.log.warning("RACECAR_BACKEND", "Map cannot be changed as the map does not exist");
			return "Map was not changed as map does not exist";
		}

	}

	/**
	 * Request all possible wayPoints from the BackBone through a REST Get request.
	 */
	private void loadWayPoints()
	{
		this.wayPoints.clear();
		if (this.params.isBackboneDisabled())
		{ // Temp wayPoints for when they can't be requested from back-end services.
			switch (this.currentMap)
			{
				case "zbuilding":
					this.log.info("MAP-MAN", "Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
					break;
				case "V314":
					this.log.info("MAP-MAN", "Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) -3.0, (float) -1.5, (float) 0.07, (float) 1.00));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) 1.10, (float) -1.20, (float) 0.07, (float) 1.00));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) 4.0, (float) -0.90, (float) -0.68, (float) 0.73));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) 4.54, (float) -4.49, (float) -0.60, (float) 0.80));
					break;
				case "gangV":
					this.log.info("MAN-PAN", "Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) -6.1, (float) -28.78, (float) 0.73, (float) 0.69));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -6.47, (float) -21.69, (float) 0.66, (float) 0.75));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -5.91, (float) -1.03, (float) 0.52, (float) 0.85));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) 6.09, (float) 0.21, (float) -0.04, (float) 1.00));
					break;
				default:
					log.warning("MAP-PAN", "The backbone could not be reached and there were no default wayPoints for this map");
					this.wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
			}
		} else
		{
			String jsonString = this.backboneRESTUtils.getJSON("map/stringmapjson/car"); //when the map is changed, another map needs to be manually loaded in the backbone database
			JSONUtils.isJSONValid(jsonString);
			Type typeOfWayPointArray = new TypeToken<ArrayList<WayPoint>>()
			{
			}.getType();
			ArrayList<WayPoint> wayPointsTemp = (ArrayList<WayPoint>) JSONUtils.getObject(jsonString, typeOfWayPointArray);
			for (WayPoint wayPoint : wayPointsTemp)
			{
				wayPoints.put(wayPoint.getID(), wayPoint);
				this.log.info("RACECAR_BACKEND", "Added wayPoint with ID " + wayPoint.getID() + " and coordinates " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW() + ".");
			}
		}
		this.log.info("RACECAR_BACKEND", "All possible wayPoints(" + wayPoints.size() + ") received.");
	}

	@Override
	public void parseMQTT(String topic, String message)
	{

	}
}

package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Lazy;
import org.springframework.core.io.InputStreamResource;
import org.springframework.core.io.Resource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.StreamingOutput;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

@Controller
public class MapManager implements MQTTListener
{
	private Logger log;
	private MapManagerParameters params;

	private VehicleManager vehicleManager;

	private MQTTUtils mqttUtils;
	private RESTUtils backboneRESTUtils;

	private Map<Long, WayPoint> wayPoints;
	private String currentMap;
	private String mapPath;

	@Autowired
	public MapManager(MapManagerParameters params, @Lazy VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.params = params;

		this.log.info("Getting settings from parameter object.");

		this.currentMap = params.getCurrentMap();
		this.mapPath = params.getMapPath();

		this.log.info("Starting MQTT Utils.");

		this.mqttUtils = new MQTTUtils(params.getMqttBroker(), params.getMqttUserName(), params.getMqttPassword(), this);

		this.log.info("Setting up Backbone REST Utils.");

		this.backboneRESTUtils = new RESTUtils(params.getBackboneRESTURL());

		this.log.info("Setting VehicleManager.");

		this.vehicleManager = vehicleManager;

		this.wayPoints = new HashMap<>();

		this.log.info("Loading Waypoints.");

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
	@RequestMapping(value="/carmanager/getMapName", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody String getMapName()
	{
		return this.currentMap;
	}

	/**
	 * REST GET server service to download a map's PGM file by name.
	 *
	 * @param mapname the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@RequestMapping(value="/carmanager/getMapPGM/{mapName}", method=RequestMethod.GET, produces=MediaType.APPLICATION_OCTET_STREAM)
	public @ResponseBody ResponseEntity<Resource> getMapPGM(@PathVariable("mapName") String mapName)
	{
		try
		{
			String resourcePath = this.mapPath + "/" + mapName + ".pgm";

			InputStreamResource resource = new InputStreamResource(new FileInputStream(resourcePath));
			HttpHeaders headers = new HttpHeaders();
			headers.add("content-disposition", "attachment");

			this.log.info("Serving request for " + resourcePath);

			return new ResponseEntity<>(resource, headers, HttpStatus.OK);
		}
		catch(IOException ioe)
		{
			String errorString = "Error fetching " + mapName + ".pgm.";
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		}
	}

	/**
	 * REST GET server service to get all currently used wayPoints by F1 vehicles.
	 *
	 * @return REST response of the type JSON containing all wayPoints.
	 */
	@RequestMapping(value = "/carmanager/getwaypoints", method = RequestMethod.GET, produces = MediaType.APPLICATION_JSON)
	public String getWayPoints()
	{
		return JSONUtils.objectToJSONStringWithKeyWord("wayPoints", this.wayPoints);
	}

	/**
	 * REST GET server service to download a map's YAML file by name.
	 *
	 * @param mapName the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@RequestMapping(value = "/carmanager/getMapYAML/{mapName}", method = RequestMethod.GET, produces = MediaType.APPLICATION_OCTET_STREAM)
	public @ResponseBody ResponseEntity<Resource> getMapYAML(@PathVariable("mapName") final String mapName)
	{
		try
		{
			String resourcePath = this.mapPath + "/" + mapName + ".yaml";

			InputStreamResource resource = new InputStreamResource(new FileInputStream(resourcePath));
			HttpHeaders headers = new HttpHeaders();
			headers.add("content-disposition", "attachment");

			this.log.info("Serving request for " + resourcePath);

			return new ResponseEntity<>(resource, headers, HttpStatus.OK);
		}
		catch (IOException ioe)
		{
			String errorString = "Error fetching " + mapName + ".yaml.";
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(HttpStatus.NOT_FOUND);
		}
	}

	/**
	 * Rest command that can be called to change the map used by the racecars at runtime
	 *
	 * @param mapName name of the new map
	 * @return
	 */
	@RequestMapping(value = "/carmanager/changeMap/{mapName}", method = RequestMethod.GET, produces = MediaType.TEXT_PLAIN)
	public String changeMap(@PathVariable("mapName") String mapName)
	{
		File mapFile = new File(mapPath + "/" + mapName + ".yaml");

		if (mapFile.exists() && mapFile.isFile())
		{
			this.currentMap = mapName;

			for (Iterator it = this.vehicleManager.getIdIterator(); it.hasNext(); )
			{
				int ID = (int) it.next();

				this.log.info("change map command send to vehicle with ID: " + ID);
				this.mqttUtils.publishMessage("racecar/" + ID + "/changeMap", mapName);
				loadWayPoints();
			}
			return "Command was executed to change map";
		}
		else
		{
			this.log.warn("Map cannot be changed as the map does not exist");
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
		{
			// Temp wayPoints for when they can't be requested from back-end services.
			switch (this.currentMap)
			{
				case "zbuilding":
					this.log.info("Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
					break;
				case "V314":
					this.log.info("Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) -3.0, (float) -1.5, (float) 0.07, (float) 1.00));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) 1.10, (float) -1.20, (float) 0.07, (float) 1.00));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) 4.0, (float) -0.90, (float) -0.68, (float) 0.73));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) 4.54, (float) -4.49, (float) -0.60, (float) 0.80));
					break;
				case "gangV":
					this.log.info("Loading wayPoints for " + this.currentMap);
					this.wayPoints.put((long) 46, new WayPoint(46, (float) -6.1, (float) -28.78, (float) 0.73, (float) 0.69));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -6.47, (float) -21.69, (float) 0.66, (float) 0.75));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -5.91, (float) -1.03, (float) 0.52, (float) 0.85));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) 6.09, (float) 0.21, (float) -0.04, (float) 1.00));
					break;
				case "U014":
					wayPoints.put((long) 46, new WayPoint(46, (float) 2.26, (float) 0.18, (float) -0.04, (float) -0.99));
					wayPoints.put((long) 47, new WayPoint(47, (float) 6.64, (float) 2.10, (float) 0.72, (float) 0.70));
					wayPoints.put((long) 48, new WayPoint(48, (float) 2.26, (float) 4.28, (float) -0.99, (float) 0.30));
					break;
				default:
					log.warn("The backbone could not be reached and there were no default wayPoints for this map");
					this.wayPoints.put((long) 46, new WayPoint(46, (float) 0.5, (float) 0, (float) -1, (float) 0.02));
					this.wayPoints.put((long) 47, new WayPoint(47, (float) -13.4, (float) -0.53, (float) 0.71, (float) 0.71));
					this.wayPoints.put((long) 48, new WayPoint(48, (float) -27.14, (float) -1.11, (float) -0.3, (float) 0.95));
					this.wayPoints.put((long) 49, new WayPoint(49, (float) -28.25, (float) -9.19, (float) -0.71, (float) 0.71));
			}
		}
		else
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
				this.log.info("Added wayPoint with ID " + wayPoint.getID() + " and coordinates " + wayPoint.getX() + "," + wayPoint.getY() + "," + wayPoint.getZ() + "," + wayPoint.getW() + ".");
			}
		}

		this.log.info("All possible wayPoints(" + wayPoints.size() + ") received.");
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

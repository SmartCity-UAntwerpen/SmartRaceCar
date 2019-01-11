package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MapManagerAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
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
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Map;

@Controller
public class MapManager implements MQTTListener, CoordinateRepository
{
	private Logger log;
	private Configuration configuration;

	private String currentMap;

	private MQTTUtils mqttUtils;

	private WaypointProvider waypointProvider;

	@Autowired
	public MapManager(@Qualifier("mapManager") Configuration configuration, @Autowired WaypointProvider waypointProvider)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.configuration = configuration;

		this.log.info("Initializing Map Manager...");

		MapManagerAspect mapManagerAspect = (MapManagerAspect) configuration.get(AspectType.MAP_MANAGER);
		this.currentMap = mapManagerAspect.getCurrentMap();

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to create MQTTUtils for MapManager.");
		}

		this.waypointProvider = waypointProvider;

		this.log.info("Initialized Map Manager.");
	}

	/**
	 * REST GET server service to get the currently used map.
	 *
	 * @return REST response of the type Text Plain containing the mapname.
	 */
	@RequestMapping(value="/carmanager/getmapname", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> getMapName()
	{
		return new ResponseEntity<>(this.currentMap, HttpStatus.OK);
	}

	/**
	 * REST GET server service to download a map's PGM file by name.
	 *
	 * @param mapName the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@RequestMapping(value="/carmanager/getmappgm/{mapName}", method=RequestMethod.GET, produces=MediaType.APPLICATION_OCTET_STREAM)
	public @ResponseBody ResponseEntity<Resource> getMapPGM(@PathVariable("mapName") String mapName)
	{
		try
		{
			MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);
			String resourcePath = mapManagerAspect.getMapPath() + "/" + mapName + ".pgm";

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
	 * REST GET server service to download a map's YAML file by name.
	 *
	 * @param mapName the name of the map
	 * @return REST response of the type Octet-stream containing the file.
	 */
	@RequestMapping(value = "/carmanager/getmapyaml/{mapName}", method = RequestMethod.GET, produces = MediaType.APPLICATION_OCTET_STREAM)
	public @ResponseBody ResponseEntity<Resource> getMapYAML(@PathVariable("mapName") final String mapName)
	{
		try
		{
			MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);
			String resourcePath = mapManagerAspect.getMapPath() + "/" + mapName + ".yaml";

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
	 * REST GET server service to get all currently used wayPoints by F1 vehicles.
	 *
	 * @return REST response of the type JSON containing all wayPoints.
	 */
	@RequestMapping(value="/carmanager/getwaypoints", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> getWayPoints()
	{
		Map<Long, WayPoint> waypoints = this.waypointProvider.getAll();
		return new ResponseEntity<>(JSONUtils.objectToJSONStringWithKeyWord("wayPoints", waypoints), HttpStatus.OK);
	}

	/**
	 * Rest command that can be called to change the map used by the racecars at runtime
	 *
	 * @param mapName name of the new map
	 * @return
	 */
	@RequestMapping(value = "/carmanager/changeMap/{mapName}", method = RequestMethod.GET, produces = MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> changeMap(@PathVariable("mapName") String mapName)
	{
		MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);
		File mapFile = new File(mapManagerAspect.getMapPath() + "/" + mapName + ".yaml");

		if (mapFile.exists() && mapFile.isFile())
		{
			this.currentMap = mapName;

			this.log.info("Publishing map change.");

			try
			{
				MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
				this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP, mapName);
			}
			catch (MqttException me)
			{
				this.log.error("Failed to publish map change.", me);
			}

			this.log.info("Changed current map to " + mapName);

			return new ResponseEntity<>(mapName, HttpStatus.OK);
		}
		else
		{
			String errorString = "Map cannot be changed as the map (\"" + mapName + "\") does not exist";
			this.log.warn(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}
	}

	@Override
	public Point getCoordinates(long waypointId)
	{
		MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);

		this.log.info("Fetching coordinates for waypoint " + waypointId + ".");

		if(this.waypointProvider.exists(waypointId))
		{
			return this.waypointProvider.get(waypointId);
		}
		return new Point(0, 0, 0, 0);
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

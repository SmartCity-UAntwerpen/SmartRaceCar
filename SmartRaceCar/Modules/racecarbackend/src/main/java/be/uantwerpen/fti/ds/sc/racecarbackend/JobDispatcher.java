package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

@Controller
public class JobDispatcher implements MQTTListener//todo: Get rid of this, still needed because MQTTUtils will crash if you don't provide it with a listener
{
	private Logger log;
	private JobDispatcherParameters jobDispatcherParameters;
	private MapManager mapManager;
	private VehicleManager vehicleManager;
	private MQTTUtils mqttUtils;

	@Autowired
	public JobDispatcher(JobDispatcherParameters jobDispatcherParameters, MapManager mapManager, VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.jobDispatcherParameters = jobDispatcherParameters;
		this.mapManager = mapManager;
		this.vehicleManager = vehicleManager;
		this.mqttUtils = new MQTTUtils(jobDispatcherParameters.getMqttBroker(), jobDispatcherParameters.getMqttUserName(), jobDispatcherParameters.getMqttPassword(), this);
	}

	@RequestMapping(value = "/carmanager/executeJob/{jobId}/{vehicleId}/{startId}/{endId}", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> jobRequest(@PathVariable long jobId, @PathVariable long vehicleId, @PathVariable long startId, @PathVariable long endId)
	{
		Job job = new Job(jobId, startId, endId, vehicleId);

		// Check if vehicle existsOld
		if (!this.vehicleManager.exists(vehicleId))
		{
			String errorString = "Tried to execute job on non-existent vehicle (" + vehicleId + ")";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		// Check if vehicle is occupied
		if (this.vehicleManager.get(vehicleId).getOccupied())
		{
			String errorString = "Vehicle " + vehicleId + " is currently occupied and can't accept any job requests.";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if vehicle is available
		if (!this.vehicleManager.get(vehicleId).isAvailable())
		{
			String errorString = "Vehicle " + vehicleId + " is currently not available for job requests.";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if starting waypoint existsOld
		if (!this.mapManager.existsOld(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint existsOld
		if (!this.mapManager.existsOld(endId))
		{
			String errorString = "Request job with non-existent end waypoint " + endId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		this.log.info("Received Job request for " + vehicleId + " from " + startId + " to " + endId + " (JobID: " + jobId + ")");

		Vehicle vehicle = this.vehicleManager.get(vehicleId);
		Location location = new Location(vehicleId, startId, endId, 0);

		vehicle.setJob(job);
		vehicle.setLocation(location);
		vehicle.setOccupied(true);

		this.mqttUtils.publishMessage("racecar/" + vehicleId + "/job", startId + " " + endId);

		return new ResponseEntity<>(HttpStatus.OK);
	}

	@RequestMapping(value="/{startPoint}/{endPoint}", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> costRequest(@PathVariable long startId, @PathVariable long endId)
	{
		RESTUtils racecarAPI = new RESTUtils(this.jobDispatcherParameters.getRESTCarmanagerURL());

		Boolean startExists = Boolean.parseBoolean(racecarAPI.getTextPlain("/exists/" + startId));
		Boolean endExists = Boolean.parseBoolean(racecarAPI.getTextPlain("/exists/" + endId));

		if (!startExists)
		{
			this.log.error("Requested cost for start waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		if (!endExists)
		{
			this.log.error("Requested cost for end waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		int cost = 0;

		if (!this.jobDispatcherParameters.isROSServerDisabled())
		{
			RESTUtils ROSAPI = new RESTUtils(this.jobDispatcherParameters.getROSServerURL());

			Type pointType = new TypeToken<Point>(){}.getType();
			Type costType = new TypeToken<Cost>(){}.getType();

			Point startPoint = (Point) JSONUtils.getObject(racecarAPI.getJSON("/getCoordinates/" + startId), pointType);
			Point endPoint = (Point) JSONUtils.getObject(racecarAPI.getJSON("/getCoordinates/" + endId), pointType);

			List<Point> points = new ArrayList<>(2);
			points.set(0, startPoint);
			points.set(1, endPoint);

			String costString = ROSAPI.postJSONGetJSON("calcWeight", JSONUtils.arrayToJSONString(points));
			Cost costObj = (Cost) JSONUtils.getObjectWithKeyWord(costString, costType);

			cost = costObj.getWeight();
		}
		else
		{
			cost = 5;
		}

		String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", cost);
		return new ResponseEntity<>(responseJson, HttpStatus.OK);
	}

	/**
	 * Dummy MQTT Parsing method.
	 * We only need MQTT Utils to publish, we don't sub to anything.
	 * The MQTTUtils still need a listener because...
	 * Honestly, I don't know why, I just know that it will crash if you don't give it one.
	 * I know, I know ...
	 *
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

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
	private JobTracker jobTracker;
	private MapManager mapManager;
	private VehicleManager vehicleManager;
	private NavigationManager navigationManager;
	private MQTTUtils mqttUtils;

	@Autowired
	public JobDispatcher(JobDispatcherParameters jobDispatcherParameters, JobTracker jobTracker, MapManager mapManager, VehicleManager vehicleManager, NavigationManager navigationManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.jobDispatcherParameters = jobDispatcherParameters;
		this.jobTracker = jobTracker;
		this.mapManager = mapManager;
		this.vehicleManager = vehicleManager;
		this.navigationManager = navigationManager;
		this.mqttUtils = new MQTTUtils(jobDispatcherParameters.getMqttBroker(), jobDispatcherParameters.getMqttUserName(), jobDispatcherParameters.getMqttPassword(), this);
	}

	/*
	@Deprecated
	@RequestMapping(value = "/carmanager/executeJob/{jobId}/{vehicleId}/{startId}/{endId}", method=RequestMethod.GET, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> jobRequest(@PathVariable long jobId, @PathVariable long vehicleId, @PathVariable long startId, @PathVariable long endId)
	{
		Job job = new Job(jobId, startId, endId, vehicleId);

		// Check if vehicle exists
		if (!this.vehicleManager.existsOld(vehicleId))
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

		// Check if starting waypoint exists
		if (!this.mapManager.existsOld(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint exists
		if (!this.mapManager.existsOld(endId))
		{
			String errorString = "Request job with non-existent end waypoint " + endId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		this.log.info("Received Job request for " + vehicleId + " from " + startId + " to " + endId + " (JobID: " + jobId + ")");

		Vehicle vehicle = this.vehicleManager.get(vehicleId);
		Location location = new Location(vehicleId, startId, endId);

		vehicle.setJob(job);
		vehicle.setLocation(location);
		vehicle.setOccupied(true);

		this.mqttUtils.publishMessage("racecar/" + vehicleId + "/job", startId + " " + endId);

		return new ResponseEntity<>(HttpStatus.OK);
	}
	*/

	/**
	 * REST Endpoint used to check the cost between the two points.
	 * The cost is calculated on the ROS navstack server (Usually at smartcity.ddns.net:8084)
	 * If no cars are available, a cost of infinity (Integer.MAX_VALUE) is returned
	 * @param startId
	 * @param endId
	 * @return
	 */
	@RequestMapping(value="/{startId}/{endId}", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> costRequest(@PathVariable long startId, @PathVariable long endId)
	{
		this.log.info("Received cost request for " + startId + " -> " + endId);

		if (!this.mapManager.existsOld(startId))
		{
			this.log.error("Requested cost for start waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		if (!this.mapManager.existsOld(endId))
		{
			this.log.error("Requested cost for end waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		if (this.vehicleManager.getNumVehicles() == 0)
		{
			this.log.error("Requested cost, but no vehicles are available, returning " + Integer.MAX_VALUE);
			String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", Integer.MAX_VALUE);
			return new ResponseEntity<>(responseJson, HttpStatus.OK);
		}

		int cost = 0;

		if (!this.jobDispatcherParameters.isROSServerDisabled())
		{
			RESTUtils ROSAPI = new RESTUtils(this.jobDispatcherParameters.getROSServerURL());

			Type costType = new TypeToken<Cost>(){}.getType();

			Point startPoint = this.mapManager.getCoordinates(startId);
			Point endPoint = this.mapManager.getCoordinates(endId);

			List<Point> points = new ArrayList<>();
			points.add(startPoint);
			points.add(endPoint);

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

	@RequestMapping(value="/job/execute/{startId}/{endId}/{jobId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> executeJob(@PathVariable long startId, @PathVariable long endId, @PathVariable long jobId)
	{
		this.log.info("Received Job request for " + startId + " -> " + endId + " (JobID: " + jobId + ")");

		//todo: Replace this with actual resource management
		long dummyVehicleId = 0;
		long vehicleId = dummyVehicleId;

		//todo: move to resource manager
		/*
		// Check if vehicle exists
		if (!this.vehicleManager.existsOld(vehicleId))
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
		}*/

		RESTUtils racecarAPI = new RESTUtils(this.jobDispatcherParameters.getRESTCarmanagerURL());

		// Check if starting waypoint exists
		if (!this.mapManager.existsOld(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint exists
		if (!this.mapManager.existsOld(endId))
		{
			String errorString = "Request job with non-existent end waypoint " + endId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		this.navigationManager.setLocation(vehicleId, startId);
		this.vehicleManager.setOccupied(vehicleId, true);
		this.jobTracker.addJob(jobId, vehicleId, startId, endId);

		this.mqttUtils.publishMessage("racecar/" + vehicleId + "/job", startId + " " + endId);
		return new ResponseEntity<>("starting", HttpStatus.OK);
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

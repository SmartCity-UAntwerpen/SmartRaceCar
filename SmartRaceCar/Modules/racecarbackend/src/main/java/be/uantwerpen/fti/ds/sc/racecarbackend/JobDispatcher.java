package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import java.util.NoSuchElementException;

@Controller
public class JobDispatcher implements MQTTListener//todo: Get rid of this, still needed because MQTTUtils will crash if you don't provide it with a listener
{
	private Logger log;
	private JobTracker jobTracker;
	private WaypointValidator waypointValidator;
	private VehicleManager vehicleManager;
	private NavigationManager navigationManager;
	private ResourceManager resourceManager;
	private MQTTUtils mqttUtils;

	@Autowired
	public JobDispatcher(@Qualifier("backend") BackendParameters backendParameters, JobTracker jobTracker, WaypointValidator waypointValidator, VehicleManager vehicleManager, NavigationManager navigationManager, ResourceManager resourceManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.jobTracker = jobTracker;
		this.waypointValidator = waypointValidator;
		this.vehicleManager = vehicleManager;
		this.navigationManager = navigationManager;
		this.resourceManager = resourceManager;
		this.mqttUtils = new MQTTUtils(backendParameters.getMqttBroker(), backendParameters.getMqttUserName(), backendParameters.getMqttPassword(), this);
	}

	@RequestMapping(value="/job/execute/{startId}/{endId}/{jobId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> executeJob(@PathVariable long startId, @PathVariable long endId, @PathVariable long jobId)
	{
		this.log.info("Received Job request for " + startId + " -> " + endId + " (JobID: " + jobId + ")");

		long vehicleId = 0;

		try
		{
			vehicleId = this.resourceManager.getOptimalCar(startId);
		}
		catch (NoSuchElementException nsee)
		{
			String errorString = "An error occurred while determining the optimal car for a job.";
			this.log.error(errorString, nsee);
			return new ResponseEntity<>(errorString, HttpStatus.PRECONDITION_FAILED);
		}

		// Check if starting waypoint exists
		if (!this.waypointValidator.exists(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint exists
		if (!this.waypointValidator.exists(endId))
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

	@RequestMapping(value="/job/gotopoint/{destId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> goToPoint (@PathVariable long destId)
	{
		this.log.info("Received GOTO command for waypoint " + destId);

		long vehicleId = this.resourceManager.getOptimalCar(destId);

		if (!this.waypointValidator.exists(destId))
		{
			String errorString = "Tried to send vehicle to non-existent waypoint " + destId + ".";
			this.log.error(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		long vehicleLocation = this.navigationManager.getLocation(vehicleId);

		//todo: find a way to track this job, without conflicting with the backbone's job ID's
		this.mqttUtils.publishMessage("racecar/" + vehicleId + "/job", vehicleLocation + " " + destId);

		return new ResponseEntity<>(HttpStatus.OK);
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

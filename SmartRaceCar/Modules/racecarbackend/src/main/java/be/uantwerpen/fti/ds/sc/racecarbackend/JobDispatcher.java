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
import java.io.IOException;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Controller
public class JobDispatcher implements MQTTListener//todo: Get rid of this, still needed because MQTTUtils will crash if you don't provide it with a listener
{
	private static final String ROUTE_UPDATE_DONE = "done";

	private static class MQTTConstants
	{
		private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
	}

	private Logger log;
	private Queue<Job> globalJobQueue;
	private Queue<Job> localJobQueue;
	private JobTracker jobTracker;
	private WaypointValidator waypointValidator;
	private VehicleManager vehicleManager;
	private LocationRepository locationRepository;
	private ResourceManager resourceManager;
	private MQTTUtils mqttUtils;

	private boolean isRouteUpdate(String topic)
	{
		Matcher matcher = JobDispatcher.MQTTConstants.ROUTE_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private void checkJobQueue()
	{
		if (!this.localJobQueue.isEmpty())
		{
			this.scheduleJob(this.localJobQueue.remove(), JobType.LOCAL);
		}
		else if (!this.globalJobQueue.isEmpty())
		{
			this.scheduleJob(this.globalJobQueue.remove(), JobType.GLOBAL);
		}
	}

	private void scheduleJob(Job job, JobType type)
	{
		this.locationRepository.setLocation(job.getVehicleId(), job.getStartId());  //todo: remove this, its fishy
		this.vehicleManager.setOccupied(job.getVehicleId(), true);

		switch (type)
		{
			case LOCAL:
				this.jobTracker.addLocalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId());
				break;

			case GLOBAL:
				this.jobTracker.addGlobalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId());
				break;
		}

		this.mqttUtils.publishMessage("racecar/" + job.getVehicleId() + "/job", job.getStartId() + " " + job.getEndId());
	}

	@Autowired
	public JobDispatcher(@Qualifier("backend") BackendParameters backendParameters, JobTracker jobTracker, WaypointValidator waypointValidator, VehicleManager vehicleManager, LocationRepository locationRepository, ResourceManager resourceManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.localJobQueue = new LinkedList<>();
		this.globalJobQueue = new LinkedList<>();
		this.jobTracker = jobTracker;
		this.waypointValidator = waypointValidator;
		this.vehicleManager = vehicleManager;
		this.locationRepository = locationRepository;
		this.resourceManager = resourceManager;
		this.mqttUtils = new MQTTUtils(backendParameters.getMqttBroker(), backendParameters.getMqttUserName(), backendParameters.getMqttPassword(), this);
	}

	@RequestMapping(value="/job/execute/{startId}/{endId}/{jobId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> executeJob(@PathVariable long startId, @PathVariable long endId, @PathVariable long jobId)
	{
		this.log.info("Received Job request for " + startId + " -> " + endId + " (JobID: " + jobId + ")");

		if (this.vehicleManager.getNumVehicles() == 0)
		{
			this.globalJobQueue.add(new Job(jobId, startId, endId, -1));
		}

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
		catch (IOException ioe)
		{
			String errorString = "An IOException was thrown while trying to find the optimal car for a job.";
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
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

		Job job = new Job(jobId, startId, endId, vehicleId);
		this.scheduleJob(job, JobType.GLOBAL);

		return new ResponseEntity<>("starting", HttpStatus.OK);
	}

	@RequestMapping(value="/job/gotopoint/{destId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> goToPoint (@PathVariable long destId)
	{
		this.log.info("Received GOTO command for waypoint " + destId);
		long vehicleId = -1;

		try
		{
			vehicleId = this.resourceManager.getOptimalCar(destId);
		}
		catch (NoSuchElementException nsee)
		{
			String errorString = "An error occurred while determining the optimal car for a go-to command.";
			this.log.error(errorString, nsee);
			return new ResponseEntity<>(errorString, HttpStatus.PRECONDITION_FAILED);
		}
		catch (IOException ioe)
		{
			String errorString = "An IOException was thrown while trying to find the optimal car for a go-to command.";
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		if (!this.waypointValidator.exists(destId))
		{
			String errorString = "Tried to send vehicle to non-existent waypoint " + destId + ".";
			this.log.error(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		long vehicleLocation = this.locationRepository.getLocation(vehicleId);

		Job job = new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, vehicleId);
		this.scheduleJob(job, JobType.GLOBAL);

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
		long vehicleId = TopicUtils.getCarId(topic);

		if (this.isRouteUpdate(topic) && (message.equals(ROUTE_UPDATE_DONE)))
		{
			this.log.info("Vehicle " + vehicleId + " completed its job. Checking for other queued jobs.");
			this.checkJobQueue();
		}
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
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
import java.util.List;
import java.util.NoSuchElementException;

@Controller
public class JobDispatcher implements MQTTListener  //todo: Get rid of this, still needed because MQTTUtils will crash if you don't provide it with a listener
{
	private static final String ROUTE_UPDATE_DONE = "done";
	private static final String MQTT_POSTFIX = "route/#";

	private Logger log;
	private List<Job> globalJobQueue;
	private List<Job> localJobQueue;
	private JobTracker jobTracker;
	private WaypointValidator waypointValidator;
	private VehicleManager vehicleManager;
	private LocationRepository locationRepository;
	private ResourceManager resourceManager;
	private MQTTUtils mqttUtils;

	private void checkJobQueue() throws IOException
	{
		if (!this.localJobQueue.isEmpty())
		{
			this.scheduleJob(this.localJobQueue.remove(0), JobType.LOCAL);
		}
		else if (!this.globalJobQueue.isEmpty())
		{
			this.scheduleJob(this.globalJobQueue.remove(0), JobType.GLOBAL);
		}
	}

	private void scheduleJob(Job job, JobType type) throws IOException
	{
		this.vehicleManager.setOccupied(job.getVehicleId(), true);

		if (job.getVehicleId() == -1)
		{
			long vehicleId = this.resourceManager.getOptimalCar(job.getStartId());
			job.setVehicleId(vehicleId);
		}

		switch (type)
		{
			case LOCAL:
				this.jobTracker.addLocalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId());
				break;

			case GLOBAL:
				this.jobTracker.addGlobalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId());
				break;
		}

		try
		{
			this.mqttUtils.publish("racecar/job/" + job.getVehicleId(), job.getStartId() + " " + job.getEndId());
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish job " + job.getJobId(), me);
		}
	}

	@Autowired
	public JobDispatcher(@Qualifier("jobDispatcher") Configuration configuration, JobTracker jobTracker, WaypointValidator waypointValidator, VehicleManager vehicleManager, LocationRepository locationRepository, ResourceManager resourceManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.localJobQueue = new LinkedList<>();
		this.globalJobQueue = new LinkedList<>();
		this.jobTracker = jobTracker;
		this.waypointValidator = waypointValidator;
		this.vehicleManager = vehicleManager;
		this.locationRepository = locationRepository;
		this.resourceManager = resourceManager;

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + MQTT_POSTFIX);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to start MQTTUtils for JobDispatcher.", me);
		}
	}

	public boolean isInQueue(long jobId, JobType type)
	{
		switch (type)
		{
			case GLOBAL:
				for (Job globalJob: this.globalJobQueue)
				{
					if (globalJob.getJobId() == jobId)
					{
						return true;
					}
				}

				return false;

			case LOCAL:
				for (Job localJob: this.localJobQueue)
				{
					if (localJob.getJobId() == jobId)
					{
						return true;
					}
				}

				return false;
		}

		String errorString = "Failed to check if job " + jobId + " (Type: " + type + ") was enqueued.";
		this.log.error(errorString);
		throw new NoSuchElementException(errorString);
	}

	/*
	 *
	 *  REST Endpoints
	 *
	 */

	@RequestMapping(value="/job/execute/{startId}/{endId}/{jobId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> executeJob(@PathVariable long startId, @PathVariable long endId, @PathVariable long jobId)
	{
		this.log.info("Received Job request for " + startId + " -> " + endId + " (JobID: " + jobId + ")");

		if (this.jobTracker.exists(jobId))
		{
			String errorString = "A job with ID " + jobId + " already exists.";
			this.log.error(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		if (this.resourceManager.getNumAvailableCars() == 0)
		{
			this.log.info("There are currently no vehicles available, adding to global queue (No. " + (this.globalJobQueue.size() + 1) + " in line.)");
			this.globalJobQueue.add(new Job(jobId, startId, endId, -1));
			return new ResponseEntity<>("starting", HttpStatus.OK);
		}

		if (!this.globalJobQueue.isEmpty())
		{
			this.log.info("There's already " + this.globalJobQueue.size() + " jobs in the global queue, adding to global queue.");
			this.globalJobQueue.add(new Job(jobId, startId, endId, -1));
			return new ResponseEntity<>("starting", HttpStatus.OK);
		}

		long vehicleId = -1;

		try
		{
			vehicleId = this.resourceManager.getOptimalCar(startId);
		}
		catch (NoSuchElementException nsee)
		{
			String errorString = "An error occurred while determining the optimal car for a job.";
			this.log.error(errorString, nsee);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
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

		try
		{
			this.scheduleJob(job, JobType.GLOBAL);
		}
		catch (IOException ioe)
		{
			String errorString = "Failed to schedule global job " + jobId;
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		return new ResponseEntity<>("starting", HttpStatus.OK);
	}

	@RequestMapping(value="/job/gotopoint/{destId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> goToPoint (@PathVariable long destId)
	{
		this.log.info("Received GOTO command for waypoint " + destId);

		long vehicleId = -1;
		long vehicleLocation = -1;

		try
		{
			vehicleId = this.resourceManager.getOptimalCar(destId);
			vehicleLocation = this.locationRepository.getLocation(vehicleId);
		}
		catch (NoSuchElementException nsee)
		{
			// If the exception was caused because no cars are available, enqueue the job
			if (this.resourceManager.getNumAvailableCars() == 0)
			{
				this.log.info("There are currently no vehicles available, adding to local queue (No. " + (this.localJobQueue.size() + 1) + " in line.)");
				this.localJobQueue.add(new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, -1));
				return new ResponseEntity<>("starting", HttpStatus.OK);
			}

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

		if (!this.globalJobQueue.isEmpty())
		{
			this.log.info("There's already " + this.localJobQueue.size() + " jobs in the local queue, adding to local queue.");
			this.localJobQueue.add(new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, -1));
			return new ResponseEntity<>("starting", HttpStatus.OK);
		}

		if (!this.waypointValidator.exists(destId))
		{
			String errorString = "Tried to send vehicle to non-existent waypoint " + destId + ".";
			this.log.error(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		Job job = new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, vehicleId);

		try
		{
			this.scheduleJob(job, JobType.LOCAL);
		}
		catch (IOException ioe)
		{
			String errorString = "Failed to schedule local job " + job.getJobId();
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

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

		if (message.equals(ROUTE_UPDATE_DONE))
		{
			this.log.info("Vehicle " + vehicleId + " completed its job. Checking for other queued jobs.");

			try
			{
				this.checkJobQueue();
			}
			catch (IOException ioe)
			{
				String errorString = "An error occurred while checking the job queue.";
				this.log.error(errorString);
			}
		}
	}
}

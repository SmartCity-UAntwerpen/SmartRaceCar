package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
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
import java.util.NoSuchElementException;

@Controller
public class JobDispatcher implements MQTTListener
{
	private static final String ROUTE_UPDATE_DONE = "done";

	private Logger log;
	private Configuration config;
	private JobTracker jobTracker;
	private JobQueue jobQueue;
	private WaypointValidator waypointValidator;
	private OccupationRepository occupationRepository;
	private LocationRepository locationRepository;
	private ResourceManager resourceManager;
	private MQTTUtils mqttUtils;
	private MessageQueueClient messageQueueClient;

	private boolean isRouteUpdate(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.config.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + Messages.CORE.ROUTE);
	}

	private boolean isRegistrationComplete(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.config.get(AspectType.MQTT);
		return topic.startsWith(mqttAspect.getTopic() + Messages.BACKEND.REGISTRATION_DONE);
	}

	private void checkJobQueue() throws IOException
	{
		if (!this.jobQueue.isEmpty(JobType.LOCAL))
		{
			this.scheduleJob(this.jobQueue.dequeue(JobType.LOCAL), JobType.LOCAL);
		}
		else if (!this.jobQueue.isEmpty(JobType.GLOBAL))
		{
			this.scheduleJob(this.jobQueue.dequeue(JobType.GLOBAL), JobType.GLOBAL);
		}
	}

	private void scheduleJob(Job job, JobType type) throws IOException
	{
		this.occupationRepository.setOccupied(job.getVehicleId(), true);

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
			MqttAspect mqttAspect = (MqttAspect) this.config.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + Messages.BACKEND.JOB + "/" + job.getVehicleId(), job.getStartId() + " " + job.getEndId());
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish job " + job.getJobId(), me);
		}
	}

	@Autowired
	public JobDispatcher(@Qualifier("jobDispatcher") Configuration configuration, JobTracker jobTracker, JobQueue jobQueue, WaypointValidator waypointValidator, OccupationRepository occupationRepository, LocationRepository locationRepository, ResourceManager resourceManager)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.config = configuration;
		this.jobQueue = jobQueue;
		this.jobTracker = jobTracker;
		this.waypointValidator = waypointValidator;
		this.occupationRepository = occupationRepository;
		this.locationRepository = locationRepository;
		this.resourceManager = resourceManager;

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + Messages.CORE.ROUTE + "/#");
		}
		catch (MqttException me)
		{
			this.log.error("Failed to start MQTTUtils for JobDispatcher.", me);
		}

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + Messages.BACKEND.REGISTRATION_DONE + "/#");
		}
		catch (Exception e)
		{
			this.log.error("Failed to start MessageQueueClient for JobDispatcher.", e);
		}
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
			this.log.info("There are currently no vehicles available, adding to global queue");
			this.jobQueue.enqueue(new Job(jobId, startId, endId, -1), JobType.GLOBAL);
			return new ResponseEntity<>("starting", HttpStatus.OK);
		}

		if (!this.jobQueue.isEmpty(JobType.GLOBAL))
		{
			this.log.info("There are already jobs in the global queue, adding to global queue.");
			this.jobQueue.enqueue(new Job(jobId, startId, endId, -1), JobType.GLOBAL);
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
				this.log.info("There are currently no vehicles available, adding to local queue.");
				this.jobQueue.enqueue(new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, -1), JobType.LOCAL);
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

		if (!this.jobQueue.isEmpty(JobType.LOCAL))
		{
			this.log.info("There already are jobs in the local queue, adding to local queue.");
			this.jobQueue.enqueue(new Job(this.jobTracker.generateLocalJobId(), vehicleLocation, destId, -1), JobType.LOCAL);
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
	 * MQTT Parsing method.
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
		long vehicleId = TopicUtils.getCarId(topic);

		if (this.isRouteUpdate(topic))
		{
			if (message.equals(ROUTE_UPDATE_DONE))
			{
				this.log.info("Vehicle " + vehicleId + " completed its job. Checking for other queued jobs.");

				this.occupationRepository.setOccupied(vehicleId, false);

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
		else if (this.isRegistrationComplete(topic))
		{
			try
			{
				Thread.sleep(500);
			}
			catch (InterruptedException ie)
			{
				ie.printStackTrace();
			}
			if (!this.jobQueue.isEmpty(JobType.LOCAL))
			{
				try
				{
					this.log.info("Dispatching a local job to newly registered vehicle (" + vehicleId + ").");
					Job job = this.jobQueue.dequeue(JobType.LOCAL);
					job.setVehicleId(this.resourceManager.getOptimalCar(job.getStartId()));
					this.scheduleJob(job, JobType.LOCAL);
				}
				catch (IOException ioe)
				{
					String errorString = "Failed to schedule local job for newly registered vehicle (" + vehicleId + ").";
					this.log.error(errorString, ioe);
				}
			}
			else if (!this.jobQueue.isEmpty(JobType.GLOBAL))
			{
				try
				{
					this.log.info("Dispatching a global job to newly registered vehicle (" + vehicleId + ").");
					Job job = this.jobQueue.dequeue(JobType.GLOBAL);
					job.setVehicleId(this.resourceManager.getOptimalCar(job.getStartId()));
					this.scheduleJob(job, JobType.GLOBAL);
				}
				catch (IOException ioe)
				{
					String errorString = "Failed to schedule global job for newly registered vehicle (" + vehicleId + ").";
					this.log.error(errorString, ioe);
				}
			}
		}
	}
}

package be.uantwerpen.fti.ds.sc.racecarbackend.jobs;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import be.uantwerpen.fti.ds.sc.racecarbackend.*;
import be.uantwerpen.fti.ds.sc.racecarbackend.maps.WaypointProvider;
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
import java.util.concurrent.*;

@Controller
public class JobDispatcher implements MQTTListener
{
	private Logger log;
	private Configuration config;
	private JobTracker jobTracker;
	private JobQueue jobQueue;
	private JobQueue lockedQueue;
	private WaypointProvider waypointProvider;
	private OccupationRepository occupationRepository;
	private LocationRepository locationRepository;
	private ResourceManager resourceManager;
	private MQTTUtils mqttUtils;
	private TopicParser topicParser;
	private MessageQueueClient messageQueueClient;
	private ConcurrentMap<Long, Job> lockedJobs = new ConcurrentHashMap<>();;        // Map containing jobs mapped to their locked vehicles id's actually bussy jobs
	private int retryDelay = 5; // time before retrying to find a vehicle for an alert within eta

	private void checkJobQueue() throws IOException
	{
		if (!this.jobQueue.isEmpty(JobType.LOCAL)&&this.resourceManager.getNumAvailableCars()!=0)
		{
			this.scheduleJob(this.jobQueue.dequeue(JobType.LOCAL), JobType.LOCAL);
		}
		else if (!this.jobQueue.isEmpty(JobType.GLOBAL)&&this.resourceManager.getNumAvailableCars()!=0)
		{
			this.scheduleJob(this.jobQueue.dequeue(JobType.GLOBAL), JobType.GLOBAL);
		}
	}

	private void scheduleJob(Job job, JobType type) throws IOException
	{
		if (job.getVehicleId() == -1)
		{
			try
			{
				long vehicleId = this.resourceManager.getOptimalCar(job.getStartId());
				job.setVehicleId(vehicleId);
			}
			catch (NoSuchElementException nsee)
			{
				this.log.error("Failed to find optimal car for job " + job.getJobId(), nsee);
			}
		}

		this.occupationRepository.setOccupied(job.getVehicleId(), true);


		switch (type)
		{
			case LOCAL:
				this.jobTracker.addLocalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId(),job.getAlert());
				if (job.getAlert()){
					lockedJobs.put(job.getVehicleId(),job);
					final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
					executorService.schedule(removeLockedJobAfterDelay(job.getVehicleId()), (long)( this.resourceManager.getJobCost(job.getStartId(),job.getEndId())+jobTracker.getAlertUnlockDelay()+5), TimeUnit.SECONDS);
				}
				break;

			case GLOBAL:
				this.jobTracker.addGlobalJob(job.getJobId(), job.getVehicleId(), job.getStartId(), job.getEndId());
				if (lockedJobs.isEmpty()){

				}else if(lockedJobs.containsKey(job.getVehicleId())){
					lockedJobs.remove(job.getVehicleId());
				}
				break;
		}

		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.config.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.JOB + "/" + job.getVehicleId() + "/" + job.getJobId(), job.getStartId() + " " + job.getEndId());
		}
		catch (MqttException me)
		{
			this.log.error("Failed to publish job " + job.getJobId(), me);
		}
	}

	private Runnable removeLockedJobAfterDelay(final long vehicleId){
		Runnable delayedRunnable = new Runnable(){
			public void run(){
				if (lockedJobs.containsKey(vehicleId)){
					lockedJobs.remove(vehicleId);
				}
				try {
					if (resourceManager.getNumAvailableCars()!=0) {
						checkJobQueue();
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		};
		return delayedRunnable;
	}

	@Autowired
	public JobDispatcher(@Qualifier("jobDispatcher") Configuration configuration, JobTracker jobTracker, JobQueue jobQueue, WaypointProvider waypointProvider, OccupationRepository occupationRepository, LocationRepository locationRepository, ResourceManager resourceManager, TopicParser topicParser)
	{
		this.log = LoggerFactory.getLogger(this.getClass());
		this.config = configuration;
		this.topicParser = topicParser;
		this.jobQueue = jobQueue;
		this.jobTracker = jobTracker;
		this.waypointProvider = waypointProvider;
		this.occupationRepository = occupationRepository;
		this.locationRepository = locationRepository;
		this.resourceManager = resourceManager;

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE + "/#");
		}
		catch (MqttException me)
		{
			this.log.error("Failed to start MQTTUtils for JobDispatcher.", me);
		}

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.REGISTRATION_DONE + "/#");
		}
		catch (Exception e)
		{
			this.log.error("Failed to start MessageQueueClient for JobDispatcher.", e);
		}
	}

	private Runnable retryDelay(final long jobId,final long startId, final long eta){
		Runnable delayedRunnable = new Runnable(){
			public void run(){
				queueOptimalJobWithinETA(jobId,startId,eta-retryDelay);
			}
		};
		return delayedRunnable;
	}

	public ResponseEntity<String> queueOptimalJobWithinETA(long jobId,long startId,long eta){
		if(eta<=0){
			String errorString = "queueing within eta is no longer possible";
			this.log.info(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		long vehicleId = -1;
		try
		{
			vehicleId = this.resourceManager.getOptimalCarWithinETA(startId,eta);
			this.log.info("Optimal car has id: "+vehicleId);

			if(vehicleId==-1){
				this.log.info("Optimal car has to wait too long, retrying in "+retryDelay);
				final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
				executorService.schedule(retryDelay(jobId, startId,eta), retryDelay, TimeUnit.SECONDS);
				String errorString = "Optimal car has to wait too long, retrying in "+retryDelay;
				return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);			}
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
		if (!this.waypointProvider.exists(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		Job alertedJob = new Job(jobId, resourceManager.getVehicleLocation(vehicleId), startId, vehicleId);
		alertedJob.setAlert(true);
		try
		{
			this.scheduleJob(alertedJob, JobType.LOCAL);

		}
		catch (IOException ioe)
		{
			String errorString = "Failed to schedule local job " + jobId;
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		return new ResponseEntity<>("Car will arrive soon", HttpStatus.OK);
	}

	/*
	 *
	 *  REST Endpoints
	 *
	 */
	/**
	 * Alert for future jobs, if possible a car will be positioned on the start-point. in the given eta seconds the job request will arrive. Goal of this function is to have a car here at the eta time if possible.
	 */
	@RequestMapping(value="/job/alert/{startId}/{jobId}/{eta}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> jobAlert(@PathVariable long startId, @PathVariable long jobId, @PathVariable long eta)
	{
		this.log.info("Received Alert for future job with jobID: "+jobId+ "at position " +startId+ " in ETA "+eta+" seconds");
		if (this.jobTracker.exists(jobId))
		{
			String errorString = "A job with ID " + jobId + " already exists. the alert may have been delayed, ignoring this alert.";
			this.log.error(errorString);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}

		if (this.resourceManager.getNumAvailableCars() == 0)
		{
			this.log.info("There are currently no vehicles available, retrying in: "+retryDelay);
			queueOptimalJobWithinETA(jobId,startId,eta);
			return new ResponseEntity<>("No vehicle available, if any would become available within the eta the alert will be executed", HttpStatus.OK);
		}

		if (!this.jobQueue.isEmpty(JobType.LOCAL))
		{
			this.log.info("There are already jobs in the local queue, retrying in: "+retryDelay);
			queueOptimalJobWithinETA(jobId,startId,eta);
			return new ResponseEntity<>("added to local queue", HttpStatus.OK);
		}

		return queueOptimalJobWithinETA(jobId,startId,eta);
	}

	@RequestMapping(value="/job/execute/{startId}/{endId}/{jobId}", method=RequestMethod.POST, produces=MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> executeJob(@PathVariable long startId, @PathVariable long endId, @PathVariable long jobId) throws CheckedIndexOutOfBoundsException {
		this.log.info("Received Job request for " + startId + " -> " + endId + " (JobID: " + jobId + ")");

		if (this.jobTracker.exists(jobId))
		{
			if(this.jobTracker.findJobType(jobId) == JobType.GLOBAL){
				String errorString = "A job with ID " + jobId + " already exists.";
				this.log.error(errorString);
				return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
			}
			Job localJob = this.jobTracker.getJob(jobId,JobType.LOCAL);

			//If it is an alerted job then remove it and use the locked vehicle
			if (this.jobTracker.getJob(jobId,JobType.LOCAL).getAlert()){
				long vehicleId = localJob.getVehicleId();
				this.jobTracker.removeJob(jobId, vehicleId);
				// Check if end waypoint exists
				if (!this.waypointProvider.exists(endId))
				{
					String errorString = "Request job with non-existent end waypoint " + endId + ".";
					this.log.error(errorString);
					return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
				}

				Job job = new Job(jobId, startId, endId, vehicleId);

				try
				{
					this.log.info("Found alerted job, executing job on locked vehicle");

					this.scheduleJob(job, JobType.GLOBAL);
					return new ResponseEntity<>("starting", HttpStatus.OK);

				}
				catch (IOException ioe)
				{
					String errorString = "Failed to schedule global job " + jobId;
					this.log.error(errorString, ioe);
					return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
				}
			}
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
		if (!this.waypointProvider.exists(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + startId + ".";
			this.log.error(errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint exists
		if (!this.waypointProvider.exists(endId))
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

		if (!this.waypointProvider.exists(destId))
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
		long vehicleId = this.topicParser.getVehicleId(topic);

		if (this.topicParser.isRouteUpdate(topic))
		{
			if (message.equals(MqttMessages.Messages.Core.DONE))
			{
				this.log.info("Vehicle " + vehicleId + " completed its job. Checking for other queued jobs.");
				if(!lockedJobs.containsKey(vehicleId)) {
					this.occupationRepository.setOccupied(vehicleId, false);
				}
					try {
						this.checkJobQueue();
					} catch (IOException ioe) {
						String errorString = "An error occurred while checking the job queue.";
						this.log.error(errorString);
					}
			}
		}
		else if (this.topicParser.isRegistrationComplete(topic))
		{
			try
			{
				Thread.sleep(500);
			}
			catch (InterruptedException ie)
			{
				this.log.error("Failed to wait 500ms before sending job.", ie);
			}

			if (!this.jobQueue.isEmpty(JobType.LOCAL))
			{
				try
				{
					this.log.info("Dispatching a local job to newly registered vehicle (" + vehicleId + ").");
					Job job = this.jobQueue.dequeue(JobType.LOCAL);
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

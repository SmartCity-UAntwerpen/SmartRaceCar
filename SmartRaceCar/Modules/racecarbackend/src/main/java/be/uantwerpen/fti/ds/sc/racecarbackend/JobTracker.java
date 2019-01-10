package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.BackboneAspect;
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

import javax.ws.rs.WebApplicationException;
import javax.ws.rs.core.MediaType;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;

@Controller
public class JobTracker implements MQTTListener
{
    private static final int ALMOST_DONE_PERCENTAGE = 80;
    // We need to contact the backbone if we're "almost there"
    // No concrete definition of "almost" has been given, so
    // I'm choosing one here. It's 80%.

    private Logger log;
    private Configuration configuration;
    private VehicleManager vehicleManager;
    private JobQueue jobQueue;
    private MQTTUtils mqttUtils;
    private MessageQueueClient messageQueueClient;
    private Map<Long, Job> localJobs;       // Map containing local jobs mapped to their IDs
                                            // Local jobs are jobs not present in the backbone,
                                            // they are tracked locally to send vehicles to the startpoint of jobs etc.
    private Map<Long, Job> globalJobs;      // Map containing jobs mapped to their job ID's

    private boolean isProgressUpdate(String topic)
    {
        MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
        return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.PERCENTAGE);
    }

    private boolean isRouteUpdate(String topic)
    {
        MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
        return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE);
    }

    private boolean isDeletion(String topic)
    {
	    MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
	    return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.DELETE);
    }

    private Job getJob(long jobId, JobType type) throws IndexOutOfBoundsException
    {
        switch (type)
        {
            case LOCAL:
                return this.localJobs.get(jobId);

            case GLOBAL:
                return this.globalJobs.get(jobId);
        }

        throw new IndexOutOfBoundsException(type.toString() + " job with ID " + jobId + " doesn't exist.");
    }

    private void vehicleDeleted(long vehicleId)
    {
	    Iterator<Long> localJobIterator = this.localJobs.keySet().iterator();

	    while(localJobIterator.hasNext())
	    {
		    long jobId = localJobIterator.next();
		    Job job = this.localJobs.get(jobId);

		    if (job.getVehicleId() == vehicleId)
		    {
			    this.log.warn("Re-queueing local job " + jobId);
			    this.localJobs.remove(jobId);
			    job.setVehicleId(-1);
			    this.jobQueue.enqueue(job, JobType.LOCAL);
		    }
	    }

	    Iterator<Long> globalJobIterator = this.globalJobs.keySet().iterator();

	    while(globalJobIterator.hasNext())
	    {
		    long jobId = globalJobIterator.next();
		    Job job = this.globalJobs.get(jobId);

		    if (job.getVehicleId() == vehicleId)
		    {
			    this.log.warn("Re-queueing global job " + jobId);
			    this.globalJobs.remove(jobId);
			    job.setVehicleId(-1);
			    this.jobQueue.enqueue(job, JobType.GLOBAL);
		    }
	    }
    }

    private void requeue(long jobId, JobType type)
    {
        Job job = this.getJob(jobId, type);
        job.setVehicleId(-1L);  // Reset vehicle Id so the JobDispatcher chooses a new vehicle.

        this.log.warn("Requeueing " + type.toString() + " job " + jobId);
        this.jobQueue.enqueue(job, type);
    }

    private void routeUpdateError(long jobId, long vehicleId)
    {
        JobType jobType = this.findJobType(jobId, vehicleId);

        this.log.warn("Requeueing " + jobType.toString() + " job " + jobId);
        this.requeue(jobId, jobType);
        this.removeJob(jobId, vehicleId);
        this.vehicleManager.setOccupied(vehicleId, false);
    }

    private void routeUpdateNotComplete(long jobId, long vehicleId)
    {
        JobType jobType = this.findJobType(jobId, vehicleId);

        this.log.warn("Requeueing " + jobType.toString() + " job " + jobId);
        this.requeue(jobId, jobType);
        this.removeJob(jobId, vehicleId);
    }

    private JobType findJobType(long jobId, long vehicleId) throws NoSuchElementException
    {
        if (this.localJobs.containsKey(jobId))
        {
            if (this.localJobs.get(jobId).getVehicleId() == vehicleId)
            {
                return JobType.LOCAL;
            }
        }
        else if (this.globalJobs.containsKey(jobId))
        {
            if (this.globalJobs.get(jobId).getVehicleId() == vehicleId)
            {
                return JobType.GLOBAL;
            }
        }

        throw new NoSuchElementException("Tried to find type for job " + jobId + " (Vehicle: " + vehicleId + "), but no job matched the IDs.");
    }

    private void removeJob(long jobId, long vehicleId)
    {
        switch (this.findJobType(jobId, vehicleId))
        {
            case GLOBAL:
                this.globalJobs.remove(jobId);
                break;

            case LOCAL:
                this.localJobs.remove(jobId);
                break;
        }
    }

    private void completeJob(long jobId, long vehicleId) throws WebApplicationException
    {
        this.log.debug("Completing job, setting vehicle " + vehicleId + " to unoccupied.");
        this.vehicleManager.setOccupied(vehicleId, false);

        BackboneAspect backboneAspect = (BackboneAspect) this.configuration.get(AspectType.BACKBONE);
        // We should only inform the backend if the job was a global job.
        if ((!backboneAspect.isBackboneDebug()) && (this.findJobType(jobId, vehicleId) == JobType.GLOBAL))
        {
            Job job = this.getJob(jobId, JobType.GLOBAL);

            RESTUtils backboneRESTUtil = new RESTUtils(backboneAspect.getBackboneServerUrl());

            try
            {
                if (!job.isBackboneNotified())
                {
                    this.log.info("Sending last minute \"close-by\" message to backbone.");
                    backboneRESTUtil.post("/jobs/vehiclecloseby/" + jobId);
                }
            }
            catch (WebApplicationException wae)
            {
                this.log.error("Failed to notify backbone with  \"close-by\" message.");
            }

	        this.log.debug("Informing Backbone about job completion.");

            try
            {
                backboneRESTUtil.post("/jobs/complete/" + jobId);
            }
            catch (WebApplicationException wae)
            {
                this.log.error("Failed to POST completion of job to backbone.", wae);
                throw wae;
            }
        }

        this.removeJob(jobId, vehicleId);
    }

    private void updateRoute(long jobId, long vehicleId, String mqttMessage) throws WebApplicationException
    {
        switch (mqttMessage)
        {
            case MqttMessages.Messages.Core.DONE:
                this.log.info("Vehicle " + vehicleId + " completed job " + jobId + ".");
                this.completeJob(jobId, vehicleId);
                break;

            case MqttMessages.Messages.Core.ERROR:
                this.log.warn("Vehicle " + vehicleId + " completed its route with errors.");
                this.routeUpdateError(jobId, vehicleId);
                break;

            case MqttMessages.Messages.Core.NOT_COMPLETE:
                this.log.info("Vehicle " + vehicleId + " hasn't completed its previous route yet.");
                this.routeUpdateNotComplete(jobId, vehicleId);
                break;
        }
    }

    private void updateProgress(long jobId, long vehicleId, int progress) throws WebApplicationException
    {
        JobType type = this.findJobType(jobId, vehicleId);
        Job job = null;

        switch (type)
        {
            case GLOBAL:
                job = this.globalJobs.get(jobId);
                break;

            case LOCAL:
                job = this.localJobs.get(jobId);
                break;
        }

        job.setProgress(progress);

        // Now we just need to inform the backbone if the job is "almost" complete.
        // If the job is local, the backbone is not aware of the job and we're done now
        if (type == JobType.LOCAL)
        {
            return;
        }

        BackboneAspect backboneAspect = (BackboneAspect) this.configuration.get(AspectType.BACKBONE);
        if ((!backboneAspect.isBackboneDebug()) && (!job.isBackboneNotified()) && (progress >= ALMOST_DONE_PERCENTAGE))
        {
            RESTUtils backboneRESTUtil = new RESTUtils(backboneAspect.getBackboneServerUrl());

            try
            {
                backboneRESTUtil.post("/jobs/vehiclecloseby/" + jobId);
                job.setBackboneNotified(true);
            }
            catch (WebApplicationException wae)
            {
                this.log.error("Failed to notify backbone of almost-completion of job.");
                throw  wae;
            }
        }
    }

    @Autowired
    public JobTracker(@Qualifier("jobTracker") Configuration configuration, VehicleManager vehicleManager, JobQueue jobQueue)
    {
        this.log = LoggerFactory.getLogger(JobTracker.class);
        this.configuration = configuration;
        this.vehicleManager = vehicleManager;
        this.jobQueue = jobQueue;

        this.log.info("Initializing JobTracker...");

        try
        {
            MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
            this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
            this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.ROUTE + "/#");
            this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Core.PERCENTAGE + "/#");
        }
        catch (MqttException me)
        {
            this.log.error("Failed to create MQTTUtils for JobTracker.", me);
        }

        try
        {
	        MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
	        this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
	        this.messageQueueClient.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.DELETE + "/#");
        }
        catch (MqttException me)
        {
        	this.log.error("Failed to create MessageQueueClient for JobTracker.", me);
        }
        catch (Exception e)
        {
	        this.log.error("Failed to subscribe to deletion topic for JobTracker.", e);
        }

        this.globalJobs = new HashMap<>();
        this.localJobs = new HashMap<>();

        this.log.info("Initialized JobTracker.");
    }

    public void addGlobalJob(long jobId, long vehicleId, long startId, long endId)
    {
        this.log.info("Adding new Global Job for tracking (Job ID: " + jobId + ", " + startId + " -> " + endId + ", Vehicle: " + vehicleId + ").");
        Job job = new Job(jobId, startId, endId, vehicleId);
        this.globalJobs.put(jobId, job);
    }

    public void addLocalJob(long jobId, long vehicleId, long startId, long endId)
    {
        this.log.info("Adding new Local Job for tracking (Job ID: " + jobId + ", " + startId + " -> " + endId + ", Vehicle: " + vehicleId + ").");
        Job job = new Job(jobId, startId, endId, vehicleId);
        this.localJobs.put(jobId, job);
    }

    public long generateLocalJobId()
    {
        // We iterate over i and find the first (lowest) value not present in the map
        long i = 0;
        for (i = 0; this.localJobs.containsKey(i); ++i)
        {
        }

        return i;
    }

    public boolean exists(long jobId)
    {
    	if (this.globalJobs.containsKey(jobId))
	    {
	    	return true;
	    }
    	else if (this.localJobs.containsKey(jobId))
	    {
	    	return true;
	    }

    	return false;
    }

    /*
     *
     *  REST Endpoints
     *
     */
    @RequestMapping(value="/job/getprogress/{jobId}", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
    public @ResponseBody ResponseEntity<String> getProgress(@PathVariable long jobId)
    {
        if (!this.globalJobs.containsKey(jobId))
        {
            String errorString = "Tried to query progress of job " + jobId + ", but job doesn't exist.";
            this.log.error(errorString);
            return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
        }

        if (this.jobQueue.isEnqueued(jobId, JobType.GLOBAL))
        {
            String jsonString = JSONUtils.objectToJSONStringWithKeyWord("progress", 0);
            return new ResponseEntity<>(jsonString, HttpStatus.OK);
        }

        String jsonString = JSONUtils.objectToJSONStringWithKeyWord("progress", this.globalJobs.get(jobId).getProgress());
        return new ResponseEntity<>(jsonString, HttpStatus.OK);
    }

    /*
     *
     *  MQTT Parsing
     *
     */
    /**
     *
     * @param topic   received MQTT topic
     * @param message received MQTT message string
     */
    @Override
    public void parseMQTT(String topic, String message)
    {
        // If a vehicle gets deleted, requeue all jobs associated with that vehicle
        if (this.isDeletion(topic))
        {
            long vehicleId = TopicUtils.getVehicleId(topic);

            this.log.warn("Vehicle " + vehicleId + " got deleted, re-queuing all associated jobs");

            this.vehicleDeleted(vehicleId);
        }
        else
        {
            String[] topicParts = topic.split("/");
            long vehicleId = Long.parseLong(topicParts[topicParts.length - 2]);
            long jobId = Long.parseLong(topicParts[topicParts.length - 1]);

            if (this.isProgressUpdate(topic))
            {
                int percentage = Integer.parseInt(message);
                this.log.info("Received Percentage update for vehicle " + vehicleId + ", Job: " + jobId + ", Status: " + percentage + "%.");

                try
                {
                    this.updateProgress(jobId, vehicleId, percentage);
                }
                catch (WebApplicationException wae)
                {
                    this.log.error("Failed to process job progress update for job " + jobId, wae);
                }
            }
            else if (this.isRouteUpdate(topic))
            {
                this.log.info("Received Route Update for vehicle " + vehicleId + "");

                try
                {
                    this.updateRoute(jobId, vehicleId, message);
                } catch (WebApplicationException wae)
                {
                    this.log.error("Failed to process route update for job " + jobId, wae);
                }
            }
            else
            {
                this.log.warn("Received unsupported MQTT Message: \"" +  message + "\" on topic \"" + topic + "\".");
            }
        }
    }
}

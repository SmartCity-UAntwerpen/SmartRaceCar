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
import org.springframework.context.annotation.Lazy;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;

@Controller
public class JobTracker implements MQTTListener
{
    private static final String ROUTE_UPDATE_DONE = "done";
    private static final String ROUTE_UPDATE_ERROR = "error";
    private static final String ROUTE_UPDATE_NOT_COMPLETE = "notcomplete";

    private static final String MQTT_PROGRESS_POSTFIX = "percentage/#";
    private static final String MQTT_ROUTEUPDATE_POSTFIX = "route/#";

    private static final int ALMOST_DONE_PERCENTAGE = 90;
    // We need to contact the backbone if we're "almost there"
    // No concrete definition of "almost" has been given, so
    // I'm choosing one here. It's 90%.

    private Logger log;
    private Configuration configuration;
    private VehicleManager vehicleManager;
    private JobQueue jobQueue;
    private MQTTUtils mqttUtils;
    private Map<Long, Job> localJobs;       // Map containing local jobs mapped to their IDs
                                            // Local jobs are jobs not present in the backbone,
                                            // they are tracked locally to send vehicles to the startpoint of jobs etc.
    private Map<Long, Job> globalJobs;      // Map containing jobs mapped to their job ID's

    private boolean isProgressUpdate(String topic)
    {
        MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
        return topic.startsWith(mqttAspect.getTopic() + MQTT_PROGRESS_POSTFIX);
    }

    private boolean isRouteUpdate(String topic)
    {
        MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
        return topic.startsWith(mqttAspect.getTopic() + MQTT_ROUTEUPDATE_POSTFIX);
    }

    private JobType findJobType(long jobId, long vehicleId)
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

    /**
     * Return the job being executed by the vehicle with id vehicleID.
     * Returns -1 if no job was found for the given vehicle.
     * @param vehicleID
     * @return
     */
    @Deprecated
    private long findJobByVehicleId(long vehicleID) throws NoSuchElementException
    {
        for (long jobId: this.globalJobs.keySet())
        {
            if (this.globalJobs.get(jobId).getVehicleId() == vehicleID)
            {
                return jobId;
            }
        }

        for (long jobId: this.localJobs.keySet())
        {
            if (this.localJobs.get(jobId).getVehicleId() == vehicleID)
            {
                return jobId;
            }
        }

        throw new NoSuchElementException("Failed to find job associated with vehicle " + vehicleID);
    }

    private void completeJob(long jobId, long vehicleId) throws IOException
    {
        this.log.debug("Completing job, setting vehicle " + vehicleId + " to unoccupied.");
        this.vehicleManager.setOccupied(vehicleId, false);

        BackboneAspect backboneAspect = (BackboneAspect) this.configuration.get(AspectType.BACKBONE);
        // We should only inform the backend if the job was a global job.
        if ((!backboneAspect.isBackboneDebug()) && (this.findJobType(jobId, vehicleId) == JobType.GLOBAL))
        {
            this.log.debug("Informing Backbone about job completion.");

            RESTUtils backboneRESTUtil = new RESTUtils(backboneAspect.getBackboneServerUrl());

            try
            {
                backboneRESTUtil.post("/jobs/complete/" + jobId);
            }
            catch (IOException ioe)
            {
                this.log.error("Failed to POST completion of job to backbone.", ioe);
                throw ioe;
            }
        }

        this.removeJob(jobId, vehicleId);
    }

    private void updateRoute(long jobId, long vehicleId, String mqttMessage) throws IOException
    {
        switch (mqttMessage)
        {
            case ROUTE_UPDATE_DONE:
                this.log.info("Vehicle " + vehicleId + " completed job " + jobId + ".");
                this.completeJob(jobId, vehicleId);
                break;

            case ROUTE_UPDATE_ERROR:
                this.log.info("Vehicle " + vehicleId + " completed its route with errors.");
                this.vehicleManager.setOccupied(vehicleId, false);
                this.removeJob(jobId, vehicleId);
                break;

            case ROUTE_UPDATE_NOT_COMPLETE:
                this.log.info("Vehicle " + vehicleId + " hasn't completed its route yet.");
                this.vehicleManager.setOccupied(vehicleId, true);
                break;
        }
    }

    private void updateProgress(long jobId, long vehicleId, int progress) throws IOException
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
            catch (IOException ioe)
            {
                this.log.error("Failed to notify backbone of almost-completion of job.");
                throw  ioe;
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
            this.mqttUtils.subscribe(mqttAspect.getTopic() + MQTT_PROGRESS_POSTFIX);
            this.mqttUtils.subscribe(mqttAspect.getTopic() + MQTT_ROUTEUPDATE_POSTFIX);
        }
        catch (MqttException me)
        {
            this.log.error("Failed to create MQTTUtils for JobTracker.", me);
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
        long vehicleId = TopicUtils.getCarId(topic);

        try
        {
            if (this.isProgressUpdate(topic))
            {
                long jobId = this.findJobByVehicleId(vehicleId);
                int percentage = Integer.parseInt(message);
                this.log.info("Received Percentage update for vehicle " + vehicleId + ", Job: " + jobId + ", Status: " + percentage + "%.");

                try
                {
                    this.updateProgress(jobId, vehicleId, percentage);
                }
                catch (IOException ioe)
                {
                    this.log.error("Failed to process job progress update for job " + jobId, ioe);
                }
            }
            else if (this.isRouteUpdate(topic))
            {
                long jobId = this.findJobByVehicleId(vehicleId);
                this.log.info("Received Route Update for vehicle " + vehicleId + "");

                try
                {
                    this.updateRoute(jobId, vehicleId, message);
                }
                catch (IOException ioe)
                {
                    this.log.error("Failed to process route update for job " + jobId, ioe);
                }
            }
        }
        catch (NoSuchElementException nsee)
        {
            this.log.error("Failed to find job for vehicle " + vehicleId, nsee);
        }
    }
}

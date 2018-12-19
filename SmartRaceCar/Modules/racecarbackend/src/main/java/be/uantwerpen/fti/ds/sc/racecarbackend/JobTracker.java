package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Controller;
import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Controller
public class JobTracker implements MQTTListener
{
    private Logger log;
    private BackendParameters backendParameters;
    private VehicleManager vehicleManager;
    private MQTTUtils mqttUtils;
    private Map<Long, Job> localJobs;       // Map containing local jobs mapped to their IDs
                                            // Local jobs are jobs not present in the backbone,
                                            // they are tracked locally to send vehicles to the startpoint of jobs etc.
    private Map<Long, Job> globalJobs;      // Map containing jobs mapped to their job ID's

    private static final String ROUTE_UPDATE_DONE = "done";
    private static final String ROUTE_UPDATE_ERROR = "error";
    private static final String ROUTE_UPDATE_NOT_COMPLETE = "notcomplete";

    private static final int ALMOST_DONE_PERCENTAGE = 90;
    // We need to contact the backbone if we're "almost there"
    // No concrete definition of "almost" has been given, so
    // I'm choosing one here. It's 90%.

    private static class MQTTConstants
    {
        private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
        private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
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
        }
    }

    private boolean isPercentageUpdate(String topic)
    {
        Matcher matcher = JobTracker.MQTTConstants.PERCENTAGE_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    private boolean isRouteUpdate(String topic)
    {
        Matcher matcher = JobTracker.MQTTConstants.ROUTE_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
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

    private void completeJob(long jobId, long vehicleId)
    {
        this.log.debug("Completing job, setting vehicle " + vehicleId + " to unoccupied.");
        this.vehicleManager.setOccupied(vehicleId, false);

        // We should only inform the backend if the job was a global job.
        if ((!this.backendParameters.isBackboneDisabled()) && (this.findJobType(jobId, vehicleId) == JobType.GLOBAL))
    {
            this.log.debug("Informing Backbone about job completion.");

            RESTUtils backboneRESTUtil = new RESTUtils(this.backendParameters.getBackboneRESTURL());
            backboneRESTUtil.postEmpty("/jobs/complete/" + jobId);
        }

        this.removeJob(jobId, vehicleId);
    }

    private void updateRoute(long jobId, long vehicleId, String mqttMessage)
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

    private void updateProgress(long jobId, long vehicleId, int progress)
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

        if ((!this.backendParameters.isBackboneDisabled()) && (!job.isBackboneNotified()) && (progress >= ALMOST_DONE_PERCENTAGE))
        {
            RESTUtils backboneRESTUtil = new RESTUtils(this.backendParameters.getBackboneRESTURL());
            backboneRESTUtil.postEmpty("/jobs/vehiclecloseby/" + jobId);
            job.setBackboneNotified(true);
        }
    }

    @Autowired
    public JobTracker(@Qualifier("backend") BackendParameters backendParameters, VehicleManager vehicleManager)
    {
        this.log = LoggerFactory.getLogger(JobTracker.class);
        this.backendParameters = backendParameters;
        this.vehicleManager = vehicleManager;

        this.log.info("Initializing JobTracker...");

        mqttUtils = new MQTTUtils(backendParameters.getMqttBroker(), backendParameters.getMqttUserName(), backendParameters.getMqttPassword(), this);
        mqttUtils.subscribeToTopic(backendParameters.getMqttTopic());

        this.globalJobs = new HashMap<>();
        this.localJobs = new HashMap<>();

        this.log.info("Initialized JobTracker.");
    }

    public void addGlobalJob(long jobId, long vehicleId, long startId, long endId)
    {
        this.log.info("Adding new Global Job for tracking (Job ID: " + jobId + ", " + startId + " -> " + endId + ", Vehicle: " + vehicleId + ").");
        Job job = new Job(startId, endId, vehicleId);
        this.globalJobs.put(jobId, job);
    }

    public void addLocalJob(long jobId, long vehicleId, long startId, long endId)
    {
        this.log.info("Adding new Local Job for tracking (Job ID: " + jobId + ", " + startId + " -> " + endId + ", Vehicle: " + vehicleId + ").");
        Job job = new Job(startId, endId, vehicleId);
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
        long jobId = this.findJobByVehicleId(vehicleId);
        boolean jobExists = jobId != -1L;

        if (this.isPercentageUpdate(topic))
        {
            if (!jobExists)
            {
                this.log.warn("Couldn't find job associated with vehicle " + vehicleId);
                return;
            }

            int percentage = Integer.parseInt(message);
            this.log.info("Received Percentage update for vehicle " + vehicleId + ", Job: " + jobId + ", Status: " + percentage + "%.");
            this.updateProgress(jobId, vehicleId, percentage);
        }
        else if (this.isRouteUpdate(topic))
        {
            if (!jobExists)
            {
                this.log.warn("Couldn't find job associated with vehicle " + vehicleId);
                return;
            }

            this.log.info("Received Route Update for vehicle " + vehicleId + "");
            this.updateRoute(jobId, vehicleId, message);
        }
    }
}

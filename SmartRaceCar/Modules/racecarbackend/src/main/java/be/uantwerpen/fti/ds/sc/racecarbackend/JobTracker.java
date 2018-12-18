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
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Controller
public class JobTracker implements MQTTListener
{
    private Logger log;
    private BackendParameters backendParameters;
    private VehicleManager vehicleManager;
    private MQTTUtils mqttUtils;
    private RESTUtils racecarApi;
    private Map<Long, Job> jobs;        // Map containing jobs mapped to their job ID's

    private static final String ROUTE_UPDATE_DONE = "done";
    private static final String ROUTE_UPDATE_ERROR = "error";
    private static final String ROUTE_UPDATE_NOT_COMPLETE = "notcomplete";

    private static final int ALMOST_DONE_PERCENTAGE = 90;
    // We need to contact the backbone if we're "almost there"
    // No concrete definition of "almost" has been given, so
    // I'm choosing one here. It's 90%.

    private static final Type LOCATION_TYPE = new TypeToken<Location>(){}.getType();

    private static class MQTTConstants
    {
        private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
        private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
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


    private void completeJob(long jobId, long vehicleId)
    {
        this.log.debug("Completing job, setting vehicle " + vehicleId + " to unoccupied.");
        this.vehicleManager.setOccupied(vehicleId, false);

        if (!this.backendParameters.isMaaSDisabled())
        {
            this.log.debug("Informing MaaS about job completion.");

            RESTUtils maasRESTUtils = new RESTUtils(this.backendParameters.getMaaSRESTUrl());
            maasRESTUtils.getTextPlain("completeJob/" + jobId);
        }

        if (!this.backendParameters.isBackboneDisabled())
        {
            this.log.debug("Informing Backbone about job completion.");

            RESTUtils backboneRESTUtil = new RESTUtils(this.backendParameters.getBackboneRESTURL());
            backboneRESTUtil.postEmpty("/jobs/complete/" + jobId);
        }

        this.jobs.remove(jobId);
    }

    private void updateRoute(long jobId, String mqttMessage)
    {
        long vehicleId = this.jobs.get(jobId).getVehicleId();

        switch (mqttMessage)
        {
            case ROUTE_UPDATE_DONE:
                this.log.info("Vehicle " + vehicleId + " completed job " + jobId + ".");
                this.completeJob(jobId, vehicleId);
                break;

            case ROUTE_UPDATE_ERROR:
                this.log.info("Vehicle " + vehicleId + " completed its route with errors.");
                this.vehicleManager.setOccupied(vehicleId, false);
                this.jobs.remove(jobId);
                break;

            case ROUTE_UPDATE_NOT_COMPLETE:
                this.log.info("Vehicle " + vehicleId + " hasn't completed its route yet.");
                this.vehicleManager.setOccupied(vehicleId, true);
                break;
        }
    }

    private void updateProgress(long jobId, int progress)
    {
        Job job = this.jobs.get(jobId);
        job.setProgress(progress);

        if ((!this.backendParameters.isBackboneDisabled()) && (!job.isBackboneNotified()) && (progress >= ALMOST_DONE_PERCENTAGE))
        {
            RESTUtils backboneRESTUtil = new RESTUtils(this.backendParameters.getBackboneRESTURL());
            backboneRESTUtil.postEmpty("/jobs/vehiclecloseby/" + jobId);
            job.setBackboneNotified(true);
        }
    }

    /**
     * Return the job being executed by the vehicle with id vehicleID.
     * Returns -1 if no job was found for the given vehicle.
     * @param vehicleID
     * @return
     */
    private long findJobByVehicleId(long vehicleID)
    {
        for (long jobId: this.jobs.keySet())
        {
            if (this.jobs.get(jobId).getVehicleId() == vehicleID)
            {
                return jobId;
            }
        }

        return -1L;
    }

    @Autowired
    public JobTracker(@Qualifier("backend") BackendParameters backendParameters, VehicleManager vehicleManager)
    {
        this.log = LoggerFactory.getLogger(JobTracker.class);
        this.backendParameters = backendParameters;
        this.vehicleManager = vehicleManager;

        this.log.info("Initializing JobTracker...");

        this.mqttUtils = new MQTTUtils(backendParameters.getMqttBroker(), backendParameters.getMqttUserName(), backendParameters.getMqttPassword(), this);
        this.mqttUtils.subscribeToTopic(backendParameters.getMqttTopic());

        this.racecarApi = new RESTUtils(backendParameters.getRESTCarmanagerURL());

        this.jobs = new HashMap<>();

        this.log.info("Initialized JobTracker.");
    }

    public void addJob (long jobId, long vehicleId, long startId, long endId)
    {
        this.log.info("Adding new Job for tracking (Job ID: " + jobId + ", " + startId + " -> " + endId + ", Vehicle: " + vehicleId + ").");
        Job job = new Job(startId, endId, vehicleId);
        this.jobs.put(jobId, job);
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


        if (!this.vehicleManager.exists(vehicleId))
        {
            this.log.warn("Received MQTT message from non-existent vehicle " + vehicleId);
            return;
        }

        if (this.isPercentageUpdate(topic))
        {
            if (!jobExists)
            {
                this.log.warn("Couldn't find job associated with vehicle " + vehicleId);
                return;
            }

            int percentage = Integer.parseInt(message);
            this.log.info("Received Percentage update for vehicle " + vehicleId + ", Job: " + jobId + ", Status: " + percentage + "%.");
            this.updateProgress(jobId, percentage);
        }
        else if (this.isRouteUpdate(topic))
        {
            if (!jobExists)
            {
                this.log.warn("Couldn't find job associated with vehicle " + vehicleId);
                return;
            }

            this.log.info("Received Route Update for vehicle " + vehicleId + "");
            this.updateRoute(jobId, message);
        }
    }
}

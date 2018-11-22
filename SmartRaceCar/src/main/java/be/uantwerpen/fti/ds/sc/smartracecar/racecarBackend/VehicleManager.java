package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class VehicleManager implements MQTTListener
{
    private static class MQTTConstants
    {
        private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
        private static final Pattern AVAILABILITY_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/available");
        private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
    }

    private static final Type LOCATION_TYPE = (new TypeToken<Location>(){}).getType();

    private VehicleManagerParameters parameters;
    private LogbackWrapper log;
    private MQTTUtils mqttUtils;
    private RESTUtils MaaSRestUtils;
    private NavigationManager navigationManager;
    private Map<Long, Vehicle> vehicles;

    private String getAvailabilityString(boolean available)
    {
        return available ? "Available" : "Not Available";
    }

    private boolean isPercentageUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.PERCENTAGE_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    private boolean isAvailabilityUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.AVAILABILITY_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    private boolean isRouteUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.ROUTE_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    private void updateRoute(long vehicleId, String mqttMessage)
    {
        switch (mqttMessage)
        {
            case "done":
                Vehicle vehicle = this.vehicles.get(vehicleId);
                vehicle.setOccupied(false);

                if (!this.parameters.isMaaSDisabled())
                {

                    long jobId = vehicle.getJob().getIdJob();
                    this.MaaSRestUtils.getTextPlain("completeJob/" + Long.toString(jobId));
                }

                vehicle.getLocation().setPercentage(100);

                log.info("Vehicle " + Long.toString(vehicleId) + " completed its route.");

                break;

            case "error":
                this.vehicles.get(vehicleId).setOccupied(false);
                log.info("Vehicle " + Long.toString(vehicleId) + " completed its route with errors.");
                break;

            case "notcomplete":
                this.vehicles.get(vehicleId).setOccupied(true);
                log.info("Vehicle " + Long.toString(vehicleId) + " hasn't completed its route yet.");
                break;
        }
    }

    public VehicleManager(VehicleManagerParameters parameters)
    {
        this.parameters = parameters;
        this.log = new LogbackWrapper();
        this.mqttUtils = new MQTTUtils(this.parameters.getMqttBroker(), this.parameters.getMqttUserName(), this.parameters.getMqttPassword(), this);
        this.mqttUtils.subscribeToTopic(this.parameters.getMqttTopic());
        this.MaaSRestUtils = new RESTUtils(parameters.getRESTCarmanagerURL());
        this.navigationManager = new NavigationManager(this, parameters);
        this.vehicles = new HashMap<>();
    }

    /**
     * Checks whether or not a vehicle exists.
     * @param vehicleId The id of the vehicle to be checked
     * @return
     */
    public boolean exists(long vehicleId)
    {
        return this.vehicles.containsKey(vehicleId);
    }

    /**
     * Register a vehicle with the vehicle manager
     * @param id
     * @param vehicle
     */
    public void regsiter(long id, Vehicle vehicle)
    {
        this.vehicles.put(id, vehicle);
    }

    /**
     *
     * @param vehicleId
     * @return
     * @throws Exception    When a non-existent vehicle is queried, an exception is thrown
     */
    public Vehicle get(long vehicleId) throws Exception
    {
        if (this.exists(vehicleId))
        {
            return this.vehicles.get(vehicleId);
        }
        else
        {
            throw new Exception("Tried to access vehicle that doesn't exist!");
        }
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        long id = TopicUtils.getCarId(topic);

        if ((id != -1) && (this.exists(id)))
        {
            if (this.isPercentageUpdate(topic))
            {
                //todo: Refactor JSON message to only have percentage and no other location information
                Location location = (Location) JSONUtils.getObject(message, LOCATION_TYPE);
                this.vehicles.get(id).getLocation().setPercentage(location.getPercentage());
                log.info("Received Percentage update for vehicle " + Long.toString(id) + ", Status: " + Integer.toString(location.getPercentage()) + "%.");
            }
            else if (this.isAvailabilityUpdate(topic))
            {
                boolean availability = Boolean.parseBoolean(message);
                this.vehicles.get(id).setAvailable(availability);
                log.info("Received Availability update for vehicle " + Long.toString(id) + ", Status: " + this.getAvailabilityString(availability));
            }
            else if (this.isRouteUpdate(topic))
            {
                log.info("Received Route Update for vehicle " + Long.toString(id) + "");
                this.updateRoute(id, message);
            }
        }
    }
}

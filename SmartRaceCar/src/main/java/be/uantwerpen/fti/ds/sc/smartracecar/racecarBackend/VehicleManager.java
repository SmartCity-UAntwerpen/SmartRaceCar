package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Location;
import be.uantwerpen.fti.ds.sc.smartracecar.common.LogbackWrapper;
import be.uantwerpen.fti.ds.sc.smartracecar.common.MQTTListener;
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
        //todo: Use Parameter objects
        private static final String MQTT_BROKER = "tcp://smartcity.ddns.net:1883";
        private static final String MQTT_USERNAME = "root";
        private static final String MQTT_PASSWORD = "smartcity";

        private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
        private static final Pattern AVAILABILITY_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/available");
    }

    private static final Type LOCATION_TYPE = (new TypeToken<Location>(){}).getType();

    private LogbackWrapper log;
    private NavigationManager navigationManager;
    private Map<Integer, Vehicle> vehicles;

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

    public VehicleManager()
    {
        this.log = new LogbackWrapper();
        this.navigationManager = new NavigationManager(this);
        this.vehicles = new HashMap<>();
    }

    /**
     * Checks whether or not a vehicle exists.
     * @param vehicleId The id of the vehicle to be checked
     * @return
     */
    public boolean exists(int vehicleId)
    {
        return this.vehicles.containsKey(vehicleId);
    }

    /**
     *
     * @param vehicleId
     * @return
     * @throws Exception    When a non-existent vehicle is queried, an exception is thrown
     */
    public Vehicle get(int vehicleId) throws Exception
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
        int id = TopicUtils.getCarId(topic);

        if ((id != -1) && (this.exists(id)))
        {
            if (this.isPercentageUpdate(topic))
            {
                //todo: Refactor JSON message to only have percentage and no other location information
                Location location = (Location) JSONUtils.getObject(message, LOCATION_TYPE);
                this.vehicles.get(id).getLocation().setPercentage(location.getPercentage());
                log.info("Received Percentage update for vehicle " + Integer.toString(id) + ", Status: " + Integer.toString(location.getPercentage()) + "%.");
            }
            else if (this.isAvailabilityUpdate(topic))
            {
                boolean availability = Boolean.parseBoolean(message);
                this.vehicles.get(id).setAvailable(availability);
                log.info("Received Availability update for vehicle " + Integer.toString(id) + ", Status: " + this.getAvailabilityString(availability));
            }
        }
        else
        {
            log.warning("Failed to parse MQTT message. topic: '" + topic + "', message: '" + message + "'");
        }
    }
}

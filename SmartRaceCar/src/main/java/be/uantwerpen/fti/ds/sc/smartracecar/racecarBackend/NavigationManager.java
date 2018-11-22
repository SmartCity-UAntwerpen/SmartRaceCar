package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Costs
public class NavigationManager implements MQTTListener
{
    private static class MQTTConstants
    {
        //todo: Use Parameter objects
        private static final String MQTT_BROKER = "tcp://smartcity.ddns.net:1883";
        private static final String MQTT_USERNAME = "root";
        private static final String MQTT_PASSWORD = "smartcity";

        private static final Pattern COST_ANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
        private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
    }

    private static final Type COST_TYPE = (new TypeToken<Cost>(){}).getType();

    private LogbackWrapper log;
    private MQTTUtils mqttUtils;
    private List<Cost> costList;
    private VehicleManager vehicleManager;

    private boolean isCostAnswer(String topic)
    {
        Matcher matcher = MQTTConstants.COST_ANSWER_REGEX.matcher(topic);
        return matcher.matches();
    }

    private boolean isLocationUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.LOCATION_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    public NavigationManager(VehicleManager vehicleManager)
    {
        this.log = new LogbackWrapper();
        this.mqttUtils = new MQTTUtils(MQTTConstants.MQTT_BROKER, MQTTConstants.MQTT_USERNAME, MQTTConstants.MQTT_PASSWORD, this);
        this.costList = new ArrayList<>();
        this.vehicleManager = vehicleManager;
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        int id = TopicUtils.getCarId(topic);

        // id == -1 means the topic wasn't valid
        // It's also possible that the topic was valid, but the vehicle just doesn't exist
        if ((id != -1) && (this.vehicleManager.exists(id)))
        {
            // We received an MQTT cost answer
            if (this.isCostAnswer(topic))
            {
                Cost cost = (Cost)JSONUtils.getObject("value", COST_TYPE);
                this.costList.add(cost);
            }
            // We received an MQTT location update
            else if (this.isLocationUpdate(topic))
            {
                try
                {
                    long locationId = Long.parseLong(message);
                    int percentage = this.vehicleManager.get(id).getLocation().getPercentage();
                    Location location = new Location(id, locationId, locationId, percentage);
                    this.vehicleManager.get(id).setLocation(location);
                }
                catch (Exception vehicleNotFoundException)
                {
                    log.error("Tried to update location on non-existent vehicle.");
                }
            }
        }
        else
        {
            log.warning("Failed to parse MQTT message. topic: '" + topic + "', message: '" + message + "'");
        }
    }
}
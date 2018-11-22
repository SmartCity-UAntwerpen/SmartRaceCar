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

    public NavigationManager()
    {
        this.log = new LogbackWrapper();
        this.mqttUtils = new MQTTUtils(MQTTConstants.MQTT_BROKER, MQTTConstants.MQTT_USERNAME, MQTTConstants.MQTT_PASSWORD, this);
        this.costList = new ArrayList<>();
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        int id = TopicUtils.getCarId(topic);

        if (id != -1)
        {
            if (this.isCostAnswer(topic))
            {
                //todo: Get a hold of the VehicleManager
                Cost cost = (Cost)JSONUtils.getObject("value", COST_TYPE);
                this.costList.add(cost);
            }
            else if (this.isLocationUpdate(topic))
            {
               long locationId = Long.parseLong(message);

               //todo: Get a hold of VehicleManager
               Location location = new Location(id, locationId, locationId, 0);
            }
        }
        else
        {
            log.warning("Failed to parse MQTT message. topic: '" + topic + "', message: '" + message + "'");
        }
    }
}
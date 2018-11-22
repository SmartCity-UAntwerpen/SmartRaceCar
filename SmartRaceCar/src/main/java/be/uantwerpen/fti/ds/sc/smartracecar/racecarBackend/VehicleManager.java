package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.LogbackWrapper;
import be.uantwerpen.fti.ds.sc.smartracecar.common.MQTTListener;

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
    }

    private LogbackWrapper log;
    private NavigationManager navigationManager;
    private Map<Integer, Vehicle> vehicles;

    private boolean isPercentageUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.PERCENTAGE_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    public VehicleManager()
    {
        this.log = new LogbackWrapper();
        this.navigationManager = new NavigationManager();
        this.vehicles = new HashMap<>();
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        int id = TopicUtils.getCarId(topic);

        if (id != -1)
        {
            if (this.isPercentageUpdate(topic))
            {

            }
        }
        else
        {
            log.warning("Failed to parse MQTT message. topic: '" + topic + "', message: '" + message + "'");
        }
    }
}

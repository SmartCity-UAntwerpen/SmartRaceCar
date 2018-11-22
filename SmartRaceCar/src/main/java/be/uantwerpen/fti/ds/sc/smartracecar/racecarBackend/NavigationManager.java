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
        private static final Pattern CAR_ID_REGEX = Pattern.compile("racecar/([0-9]+)/.*");
        private static final Pattern COST_ANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
        private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
    }

    private static final Type COST_TYPE = (new TypeToken<Cost>(){}).getType();

    private LogbackWrapper log;
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

    /**
     * Extracts the car's id from any topic.
     * If the topic doesn't match a racecar topic, -1 is returned
     * @param topic
     * @return
     */
    private int getCarId(String topic)
    {
        Matcher matcher = MQTTConstants.CAR_ID_REGEX.matcher(topic);

        if (matcher.matches())
        {
            // Group 0 matches the entire string, so real capture groups start at index 1
            String idString = matcher.group(1);

            try
            {
                int id = Integer.parseInt(idString);
                return id;
            }
            catch (NumberFormatException nfe)
            {
                log.error("Extracted invalid integer ('" + idString + "') from racecar topic ('" + topic + "').");
                return -1;
            }
        }
        else
        {
            log.warning("Failed to extract car id from topic: '" + topic + "'");
            return -1;
        }
    }

    public NavigationManager()
    {
        this.log = new LogbackWrapper();
        this.costList = new ArrayList<>();
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        int id = this.getCarId(topic);

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
package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.MQTTListener;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Costs
public class NavigationManager implements MQTTListener
{
    private static class MQTTConstants
    {
        private static final Pattern CAR_ID_REGEX = Pattern.compile("racecar/([0-9]+)/.*");
        private static final Pattern COSTANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
    }

    private boolean isCostAnswer(String topic)
    {
        Matcher matcher = MQTTConstants.COSTANSWER_REGEX.matcher(topic);
        return matcher.matches();
    }

    /**
     * Extracts the car's id from any topic.
     * If the topic doesn't match a racecar topic, -1 is returned
     * @param topic
     * @return
     */
    public int getCarId(String topic)
    {
        Matcher matcher = MQTTConstants.CAR_ID_REGEX.matcher(topic);

        if (matcher.matches())
        {
            int id = Integer.parseInt(matcher.group(0));
            return id;
        }
        else
        {
            return -1;
        }
    }

    @Override
    public void parseMQTT(String topic, String message)
    {
        int id = this.getCarId(topic);

        if (this.getCarId(topic) != -1)
        {

        }
        else
        {

        }
    }
}
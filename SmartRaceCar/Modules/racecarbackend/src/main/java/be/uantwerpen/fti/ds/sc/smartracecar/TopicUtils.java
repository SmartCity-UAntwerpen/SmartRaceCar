package be.uantwerpen.fti.ds.sc.smartracecar;


import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class TopicUtils
{
	private static class MQTTConstants
	{
		private static final Pattern CAR_ID_REGEX = Pattern.compile("racecar/([0-9]+)/.*");
	}

	/**
	 * Extracts the car's id from any topic.
	 * If the topic doesn't match a racecar topic, -1 is returned
	 *
	 * @param topic
	 * @return
	 */
	public static long getCarId(String topic)
	{
		LogbackWrapper log = new LogbackWrapper(TopicUtils.class);
		Matcher matcher = MQTTConstants.CAR_ID_REGEX.matcher(topic);

		if (matcher.matches())
		{
			// Group 0 matches the entire string, so real capture groups start at index 1
			String idString = matcher.group(1);

			try
			{
				long id = Long.parseLong(idString);
				return id;
			} catch (NumberFormatException nfe)
			{
				log.error("TopicUtils", "Extracted invalid integer ('" + idString + "') from racecar topic ('" + topic + "').");
				return -1;
			}
		} else
		{
			log.warning("TopicUtils", "Failed to extract car id from topic: '" + topic + "'");
			return -1;
		}
	}
}

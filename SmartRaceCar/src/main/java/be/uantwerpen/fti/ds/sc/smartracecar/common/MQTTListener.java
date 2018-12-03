package be.uantwerpen.fti.ds.sc.smartracecar.common;


/**
 * MQTT functionality interface to be called on the other modules.
 */
public interface MQTTListener
{
	/**
	 * Interfaced method to parse MQTT message and topic after MQTT callback is triggered by incoming message.
	 *
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	void parseMQTT(String topic, String message);
}

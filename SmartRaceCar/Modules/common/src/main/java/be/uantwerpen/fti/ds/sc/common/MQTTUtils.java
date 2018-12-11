package be.uantwerpen.fti.ds.sc.common;

import org.eclipse.paho.client.mqttv3.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Help model to deal with MQTT. Uses Paho library.
 * Can subscribe and publish on various topics chosen.
 */
public class MQTTUtils implements MqttCallback
{

	private Logger log;

	private MqttClient client; // The MQTT client
	private MQTTListener listener; // Through the interface this is capable of triggering methods in classes that have this interface implemented.

	/**
	 * Help model to deal with MQTT. Uses Paho library.
	 * Can subscribe and publish on various topics chosen.
	 *
	 * @param brokerURL URL to the MQTT broker server.
	 * @param username  Username to log in on the MQTT broker.
	 * @param password  Password to log in on the MQTT broker.
	 * @param listener  Interface listener to trigger methods in classes implementing the MQTT interface.
	 */
	public MQTTUtils(String brokerURL, String username, String password, MQTTListener listener)
	{
		this.log = LoggerFactory.getLogger(this.getClass());

		MqttConnectOptions options = new MqttConnectOptions();
		this.listener = listener;

		options.setCleanSession(true);
		options.setMaxInflight(100);
		options.setKeepAliveInterval(0);

		if (!username.equals("") && !password.equals(""))
		{
			options.setUserName(username);
			options.setPassword(password.toCharArray());
		}

		try
		{
			client = new MqttClient(brokerURL, MqttClient.generateClientId());
			client.setCallback(this);
			client.connectWithResult(options);
			this.log.info("Connected to '" + brokerURL + "'.");
		}
		catch (MqttException e)
		{
			this.log.error("Could not connect to '" + brokerURL + "'." + e);
			System.exit(0);
		}
	}

	/**
	 * This method is called when the connection to the server is lost.
	 *
	 * @param t the reason behind the loss of connection.
	 */
	@Override
	public void connectionLost(Throwable t)
	{
		this.log.error("Connection lost.");
		t.printStackTrace();
		System.exit(0);
	}

	/**
	 * This method is called when a message arrives from the server.
	 * This method is invoked synchronously by the MQTT client.
	 * An acknowledgment is not sent back to the server until this method returns cleanly.
	 * If an implementation of this method throws an Exception, then the client will be shut down.
	 * When the client is next re-connected, any QoS 1 or 2 messages will be redelivered by the server.
	 * Any additional messages which arrive while an implementation of this method is running,
	 * will build up in memory, and will then back up on the network.
	 * If an application needs to persist data, then it should ensure the data is persisted prior to
	 * returning from this method, as after returning from this method, the message is considered to have
	 * been delivered, and will not be reproducible It is possible to send a new message within an
	 * implementation of this callback (for example, a response to this message),
	 * but the implementation must not disconnect the client, as it will be impossible to send
	 * an acknowledgment for the message being processed, and a deadlock will occur.
	 * <p>
	 * Will trigger a link on the interface to call on the class implementing the interface to parse the message.
	 *
	 * @param topic       Name of the topic on the message was published to.
	 * @param mqttMessage The actual message.
	 * @throws Exception Throws a terminal error has occurred, and the client should be shut down.
	 */
	@Override
	public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception
	{
		String message = new String(mqttMessage.getPayload());
		this.log.info("message arrived. Topic:" + topic + " | Message:" + message);
		listener.parseMQTT(topic, message);
	}

	/**
	 * Called when delivery for a message has been completed, and all acknowledgments have been received.
	 * For QoS 0 messages it is called once the message has been handed to the network for delivery.
	 * For QoS 1 it is called when PUBACK is received and for QoS 2 when PUBCOMP is received.
	 * The token will be the same token as that returned when the message was published.
	 *
	 * @param iMqttDeliveryToken The delivery token associated with the message.
	 */
	@Override
	public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken)
	{
		this.log.info("Publish complete.");
	}

	/**
	 * Subscribe to a chosen topic.
	 *
	 * @param topic The topic to subscribe on.
	 */
	public void subscribeToTopic(String topic)
	{
		try
		{
			int subQoS = 2;
			client.subscribe(topic, subQoS);
			this.log.info("Subscribed to topic '" + topic + "'.");
		}
		catch (Exception e)
		{
			this.log.error("Could not subscribe to topic '" + topic + "'." + e);
		}
	}

	/**
	 * Publish a message on a chosen topic.
	 *
	 * @param topic   The topic to publish on.
	 * @param message The message to be published.
	 */
	public void publishMessage(String topic, String message)
	{
		MqttMessage mqttMessage = new MqttMessage(message.getBytes());
		mqttMessage.setRetained(false);
		mqttMessage.setQos(2);
		this.log.info("Publishing. Topic:" + topic + " | Message:" + message);
		MqttTopic mqttTopic = client.getTopic(topic);

		try
		{
			mqttTopic.publish(mqttMessage);
		}
		catch (Exception e)
		{
			this.log.error("Could not Publish." + e);
		}
	}

	/**
	 * Disconnect the connection of the client to the MQTT broker and close it.
	 */
	public void closeMQTT()
	{
		try
		{
			client.disconnect();
			client.close();
		}
		catch (MqttException e)
		{
			this.log.error("Could not close MQTT Connection :" + e);
		}

	}
}

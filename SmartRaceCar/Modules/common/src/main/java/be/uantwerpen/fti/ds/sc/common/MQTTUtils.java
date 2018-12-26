package be.uantwerpen.fti.ds.sc.common;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Help model to deal with MQTT. Uses Paho library.
 * Can subscribe and publish on various topics chosen.
 */
public class MQTTUtils implements MqttCallback, MessageQueueClient
{
	private static final int DEFAULT_QoS = 2;

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
	public MQTTUtils(String brokerURL, String username, String password, MQTTListener listener) throws MqttException
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
			this.client = new MqttClient(brokerURL, MqttClient.generateClientId(), new MemoryPersistence());
			this.client.setCallback(this);
			this.client.connectWithResult(options);
			this.log.info("Connected to '" + brokerURL + "'.");
		}
		catch (MqttException e)
		{
			String errorString = "Could not connect to '" + brokerURL + "': " + e.getMessage();
			this.log.error(errorString, e);
			throw e;
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
		String errorString = "Connection lost.";
		this.log.error(errorString, t);
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
		this.log.debug("message arrived. Topic:" + topic + " | Message:" + message);
		this.listener.parseMQTT(topic, message);
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
		StringBuilder logMessageBuilder = new StringBuilder();

		logMessageBuilder.append("Published \"");

		try
		{
			logMessageBuilder.append(iMqttDeliveryToken.getMessage());
		}
		catch (MqttException me)
		{
			logMessageBuilder.append("UNKNOWN MESSAGE");
		}

		logMessageBuilder.append("\"");

		String[] topics = iMqttDeliveryToken.getTopics();

		if (topics != null)
		{
			logMessageBuilder.append(" to ");

			for (int i = 0; i < topics.length; ++i)
			{
				logMessageBuilder.append(topics[i]);

				if ((i + 1) != topics.length)
				{
					logMessageBuilder.append(", ");
				}
			}
		}

		this.log.debug(logMessageBuilder.toString());
	}

	/**
	 * Subscribe to a chosen topic.
	 *
	 * @param topic The topic to subscribe on.
	 */
	public void subscribe(String topic)
	{
		try
		{
			this.client.subscribe(topic, DEFAULT_QoS);
			this.log.debug("Subscribed to topic '" + topic + "' with QoS " + DEFAULT_QoS + ".");
		}
		catch (Exception e)
		{
			this.log.error("Could not subscribe to topic '" + topic + "' with QoS " + DEFAULT_QoS + ".", e);
		}
	}

	/**
	 * Publish a message on a chosen topic.
	 *
	 * @param topic   The topic to publish on.
	 * @param message The message to be published.
	 */
	public void publish(String topic, String message) throws MqttException
	{
		MqttMessage mqttMessage = new MqttMessage(message.getBytes());
		mqttMessage.setRetained(false);
		mqttMessage.setQos(DEFAULT_QoS);
		this.log.debug("Publishing. Topic:" + topic + " | Message:" + message + " | QoS: " + DEFAULT_QoS);
		MqttTopic mqttTopic = this.client.getTopic(topic);

		try
		{
			mqttTopic.publish(mqttMessage);
		}
		catch (MqttException me)
		{
			// Catch, log and re-throw
			this.log.error("Could not Publish \"" + message + "\" to \"" + topic + "\".", me);
			throw me;
		}
	}

	/**
	 * Disconnect the connection of the client to the MQTT broker and close it.
	 */
	public void closeMQTT()
	{
		try
		{
			this.client.disconnect();
			this.client.close();
		}
		catch (MqttException e)
		{
			this.log.error("Could not close MQTT Connection :" + e);
		}
	}
}

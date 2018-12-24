package be.uantwerpen.fti.ds.sc.core;


import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Helper class where the heartbeat is published every 10 seconds in a separate thread
 */
public class HeartbeatPublisher extends Thread
{
	private static final int WAITING_PERIOD = 10000;    // Waiting period between heartbeats (in ms)

	private Logger log;

	private MQTTUtils mqttUtils;
	private long ID;

	/**
	 * Constructor for the heartbeat publisher
	 *
	 * @param mqttUtils The MQTTUtils class used by the Core class
	 * @param ID        Id of the Core class that creates this thread
	 */
	public HeartbeatPublisher(MQTTUtils mqttUtils, long ID)
	{
		this.log = LoggerFactory.getLogger(HeartbeatPublisher.class);

		this.mqttUtils = mqttUtils;
		this.ID = ID;
	}

	/**
	 * start the thread
	 */
	public void run()
	{
		while (true)
		{
			try
			{
				Thread.sleep(WAITING_PERIOD); //Sleep for 10s
				this.log.info("Publishing Heartbeat...");
				this.mqttUtils.publishMessage("racecar/heartbeat/" + this.ID , "heartbeat"); //status can also be send in this message
			}
			catch (InterruptedException ie)
			{
				this.log.error("HeartBeatPublisher got interrupted while waiting to publish.", ie);
			}
			catch (MqttException me)
			{
				this.log.error("HeartBeatPublisher failed to publish heartbeat.", me);
			}
		}
	}

}

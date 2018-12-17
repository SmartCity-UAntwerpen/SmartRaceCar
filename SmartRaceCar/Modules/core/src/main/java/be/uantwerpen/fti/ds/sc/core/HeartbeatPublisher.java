package be.uantwerpen.fti.ds.sc.core;


import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Helper class where the heartbeat is published every 10 seconds in a separate thread
 */
public class HeartbeatPublisher extends Thread
{
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
				Thread.sleep(10000); //Sleep for 10s
				this.log.info("Publishing Heartbeat...");
				this.mqttUtils.publishMessage("racecar/" + this.ID + "/heartbeat", "heartbeat"); //status can also be send in this message
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}

}

package be.uantwerpen.fti.ds.sc.smartracecar;


import java.util.Date;
import java.util.HashMap;

/**
 * Helper class where the heartbeat is published every 10 seconds in a separate thread
 */
public class HeartbeatPublisher extends Thread
{
	private LogbackWrapper log;

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
		this.log = new LogbackWrapper(HeartbeatPublisher.class);

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
				this.log.info("HEARTBEAT-PUB", "Publishing Heartbeat...");
				this.mqttUtils.publishMessage("racecar/" + this.ID + "/heartbeat", "heartbeat"); //status can also be send in this message
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}

}

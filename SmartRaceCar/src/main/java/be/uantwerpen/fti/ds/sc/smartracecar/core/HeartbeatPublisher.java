package be.uantwerpen.fti.ds.sc.smartracecar.core;


import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;
import be.uantwerpen.fti.ds.sc.smartracecar.common.MQTTUtils;

import java.util.Date;
import java.util.HashMap;

/**
 * Helper class where the heartbeat is published every 10 seconds in a separate thread
 */
public class HeartbeatPublisher extends Thread
{
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
				Log.logConfig("CORE", "Publishing Heartbeat...");
				this.mqttUtils.publishMessage("racecar/" + this.ID + "/heartbeat", "heartbeat"); //status can also be send in this message
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}

}

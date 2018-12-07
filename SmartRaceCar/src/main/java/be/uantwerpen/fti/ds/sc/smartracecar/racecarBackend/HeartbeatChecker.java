package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

//import javax.servlet.http.HttpServletResponse;
//import javax.xml.ws.Response;
import java.lang.reflect.Type;
import java.util.Date;
import java.util.HashMap;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
class HeartbeatChecker extends Thread
{
	private LogbackWrapper log;
	private RESTUtils restUtils;

	/**
	 * constructor for the HeartbeatChecker class
	 *
	 * @param url RestURL of the backend object
	 */
	public HeartbeatChecker(String url)
	{
		this.log = new LogbackWrapper();

		this.log.info("HEARTBEAT-CHECKER", "Creating REST Utils for \"" + url + "\"...");

		this.restUtils = new RESTUtils(url);
	}

	/**
	 * start the Checker thread
	 */
	public void run()
	{
		this.log.info("HEARTBEAT-CHECKER", "Heartbeatchecker was started");
		while (true)
		{
			try
			{
				Thread.sleep(30000); //Sleep for 30s
				this.log.info("HEARTBEAT-CHECKER", "Heartbeats are being checked...");
				Type typeOfHashMap = new TypeToken<HashMap<Long, Vehicle>>()
				{
				}.getType();
				HashMap<Long, Vehicle> vehicles = (HashMap<Long, Vehicle>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getVehicles"), typeOfHashMap);
				Date currentTime = new Date();
				for (Long ID : vehicles.keySet())
				{
					if ((currentTime.getTime() - vehicles.get(ID).getHeartbeat().getTime()) > 90000) //longer than 90 seconds
					{
						restUtils.getCall("delete/" + ID);
						this.log.warning("HEARTBEAT-CHECKER", "Vehicle with ID: " + ID + " was removed since it hasn't responded for over 90s");
					}
				}
				this.log.info("HEARTBEAT-CHECKER", "All heartbeats were checked.");

			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}
}

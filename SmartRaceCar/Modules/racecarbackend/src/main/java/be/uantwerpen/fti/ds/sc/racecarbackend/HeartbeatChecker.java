package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Parameters;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.lang.reflect.Type;
import java.util.Date;
import java.util.HashMap;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
@Component
class HeartbeatChecker
{
	private static final long MAX_DELTA = 90000;	// Maximum amount of time between consecutive heartbeats (in ms)

	private Logger log;
	private RESTUtils restUtils;

	@Scheduled(fixedRate = 30000)
	private void checkBeats()
	{
		this.log.info("Heartbeats are being checked...");

		Type typeOfHashMap = new TypeToken<HashMap<Long, Vehicle>>(){}.getType();
		HashMap<Long, Vehicle> vehicles = (HashMap<Long, Vehicle>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getVehicles"), typeOfHashMap);
		Date currentTime = new Date();

		for (Long ID : vehicles.keySet())
		{
			final long delta = currentTime.getTime() - vehicles.get(ID).getHeartbeat().getTime();

			this.log.debug("Vehicle " + ID + "'s last heartbeat came " + (delta / 1000) + "s ago.");

			if (delta > MAX_DELTA) //longer than 90 seconds
			{
				restUtils.getCall("delete/" + ID);
				this.log.warn("Vehicle with ID: " + ID + " was removed since it hasn't responded for over 90s");
			}
		}

		this.log.info("All heartbeats were checked.");
	}

	/**
	 * constructor for the HeartbeatChecker class
	 *
	 * @param parameters parameters used to start backend
	 */
	@Autowired
	public HeartbeatChecker(Parameters parameters)
	{
		this.log = LoggerFactory.getLogger(HeartbeatChecker.class);

		this.log.debug("Creating REST Utils for \"" + parameters.getRESTCarmanagerURL() + "\"...");

		this.restUtils = new RESTUtils(parameters.getRESTCarmanagerURL());
	}
}

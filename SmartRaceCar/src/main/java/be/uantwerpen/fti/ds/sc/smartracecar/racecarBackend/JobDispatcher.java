package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
@Controller
public class JobDispatcher implements MQTTListener//todo: Get rid of this, still needed because MQTTUtils will crash if you don't provide it with a listener
{
	private LogbackWrapper log;
	private MapManager mapManager;
	private VehicleManager vehicleManager;
	private MQTTUtils mqttUtils;

	@Autowired
	public JobDispatcher(Parameters parameters, MapManager mapManager, VehicleManager vehicleManager)
	{
		this.log = new LogbackWrapper(JobDispatcher.class);
		this.mapManager = mapManager;
		this.vehicleManager = vehicleManager;
		this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);
	}


	@RequestMapping(value = "/carmanager/executeJob.{jobId}/{vehicleId}/{startId}/{endId}", method = RequestMethod.GET, produces = MediaType.TEXT_PLAIN)
	public @ResponseBody ResponseEntity<String> jobRequest(@PathVariable("jobId") long jobId, @PathVariable("vehicleId") long vehicleId, @PathVariable("startId") long startId, @PathVariable("endId") long endId)
	{
		Job job = new Job(jobId, startId, endId, vehicleId);

		// Check if vehicle exists
		if (!this.vehicleManager.exists(vehicleId))
		{
			String errorString = "Tried to execute job on non-existent vehicle (" + Long.toString(vehicleId) + ")";
			this.log.error("JOB-DISPATCHER", errorString);

			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}

		// Check if vehicle is occupied
		if (this.vehicleManager.get(vehicleId).getOccupied())
		{
			String errorString = "Vehicle " + Long.toString(vehicleId) + " is currently occupied.";
			this.log.error("JOB-DISPATCHER", errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if vehicle is available
		if (!this.vehicleManager.get(vehicleId).isAvailable())
		{
			String errorString = "Vehicle " + Long.toString(vehicleId) + " is currently not available.";
			this.log.error("JOB-DISPATCHER", errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if starting waypoint exists
		if (!this.mapManager.exists(startId))
		{
			String errorString = "Request job with non-existent start waypoint " + Long.toString(startId) + ".";
			this.log.error("JOB-DISPATCHER", errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		// Check if end waypoint exists
		if (!this.mapManager.exists(endId))
		{
			String errorString = "Request job with non-existent end waypoint " + Long.toString(startId) + ".";
			this.log.error("JOB-DISPATCHER", errorString);

			return new ResponseEntity<>(errorString, HttpStatus.NOT_FOUND);
		}

		Vehicle vehicle = this.vehicleManager.get(vehicleId);
		Location location = new Location(vehicleId, startId, endId, 0);

		vehicle.setJob(job);
		vehicle.setLocation(location);
		vehicle.setOccupied(true);

		this.mqttUtils.publishMessage("racecar/" + Long.toString(vehicleId) + "/job", Long.toString(startId) + " " + Long.toString(endId));

		return new ResponseEntity<>(HttpStatus.OK);
	}

	/**
	 * Dummy MQTT Parsing method.
	 * We only need MQTT Utils to publish, we don't sub to anything.
	 * The MQTTUtils still need a listener because...
	 * Honestly, I don't know why, I just know that it will crash if you don't give it one.
	 * I know, I know ...
	 *
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

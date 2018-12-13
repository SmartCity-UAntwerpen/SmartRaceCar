package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class RacecarBackend
{
	private static final String DEFAULT_PROPERTIES_FILE = "RacecarBackend.properties";

	private Logger log;
	private JobDispatcher jobDispatcher;
	private MapManager mapManager;
	private VehicleManager vehicleManager;

	@Autowired
	public RacecarBackend(JobDispatcher jobDispatcher, MapManager mapManager, VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(RacecarBackend.class);
		this.jobDispatcher = jobDispatcher;
		this.mapManager = mapManager;
		this.vehicleManager = vehicleManager;
	}


}

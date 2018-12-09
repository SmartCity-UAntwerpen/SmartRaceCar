package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
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

	public static void main(String[] args)
	{
		SpringApplication.run(RacecarBackend.class, args);
	}
}

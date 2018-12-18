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
	private Logger log;
	private JobDispatcher jobDispatcher;
	private JobTracker jobTracker;
	private MapManager mapManager;
	private NavigationManager navigationManager;
	private VehicleManager vehicleManager;

	@Autowired
	public RacecarBackend(JobDispatcher jobDispatcher, JobTracker jobTracker, MapManager mapManager, NavigationManager navigationManager, VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(RacecarBackend.class);
		this.jobDispatcher = jobDispatcher;
		this.jobTracker = jobTracker;
		this.mapManager = mapManager;
		this.vehicleManager = vehicleManager;
		this.navigationManager = navigationManager;
	}

	public static void main(String[] args)
	{
		SpringApplication.run(RacecarBackend.class, args);
	}
}

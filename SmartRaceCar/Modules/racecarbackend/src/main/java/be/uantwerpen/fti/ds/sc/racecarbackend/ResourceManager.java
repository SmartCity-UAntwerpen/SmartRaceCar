package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.TreeSet;

@Service
public class ResourceManager
{
	private Logger log;
	private LocationRepository locationRepository;
	private VehicleRepository vehicleRepository;
	private OccupationRepository occupationRepository;
	private CostCache costCache;

	// Cost object used to sort according to cost, but keep association with ID
	private class Cost implements Comparable<Cost>
	{
		private long vehicleId;
		private float cost;

		public Cost (long vehicleId, float cost)
		{
			this.vehicleId = vehicleId;
			this.cost = cost;
		}

		public long getVehicleId()
		{
			return this.vehicleId;
		}

		@Override
		public int compareTo(Cost cost)
		{
			return (int)(this.cost - cost.cost);
		}
	}

	@Autowired
	public ResourceManager (CostCache costCache, LocationRepository locationRepository, VehicleRepository vehicleRepository, OccupationRepository occupationRepository)
	{
		this.log = LoggerFactory.getLogger(ResourceManager.class);

		this.log.info("Initializing ResourceManager...");
		this.locationRepository = locationRepository;
		this.vehicleRepository = vehicleRepository;
		this.occupationRepository = occupationRepository;
		this.costCache = costCache;
		this.log.info("Initialized ResourceManager.");
	}

	/**
	 * Determine how many cars are available to receive new jobs.
	 * @return
	 */
	public int getNumAvailableCars()
	{
		int numAvailableCars = 0;

		for (long vehicleId: this.vehicleRepository.getVehicleIds())
		{
			if (!this.occupationRepository.isOccupied(vehicleId))
			{
				++numAvailableCars;
			}
		}

		return numAvailableCars;
	}
	public long getVehicleLocation(long vehicleId) {
		long vehiclePosition = this.locationRepository.getLocation(vehicleId);
		return vehiclePosition;
	}

	/**
	 *  Builds upon getOptimalCar method, takes the ETA into consideration, if the optimal car doesnt have too wait for too long ( determined by the maxWaitingTime )
	 * @param waypointId,eta
	 * @return
	 */
	public long getOptimalCarWithinETA(long waypointId,long eta) throws NoSuchElementException, IOException
	{
		long maxWaitingTime = 5; //5 seconds
		long optimalVehicleId = getOptimalCar(waypointId);
		float cost = 1000;
		if (!this.occupationRepository.isOccupied(optimalVehicleId))
		{
			long vehiclePosition = this.locationRepository.getLocation(optimalVehicleId);
			cost = this.costCache.calculateCost(vehiclePosition, waypointId);
		}
		if(cost+maxWaitingTime > eta ) {
			return optimalVehicleId;
		}else{
			return -1;
		}
	}


	public float getJobCost(long startId,long endId){
		try {
			return this.costCache.calculateCost(startId,endId);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return 10;
	}
	/**
	 *  Determine which car is closest to (has the lowest cost) to get to a certain point.
	 * @param waypointId
	 * @return
	 */
	public long getOptimalCar (long waypointId) throws NoSuchElementException, IOException
	{
		if (this.vehicleRepository.getNumVehicles() == 0)
		{
			String errorString = "Requested optimal car, but no cars exist.";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		// We use a TreeSet here to keep the cost objects
		// TreeSet's keep their values in an ordered fashion by default,
		// This helps us extract the element with the least cost in the end
		TreeSet<Cost> costSet = new TreeSet<>();

		for (long vehicleId: this.vehicleRepository.getVehicleIds())
		{
			if (!this.occupationRepository.isOccupied(vehicleId))
			{
				long vehiclePosition = this.locationRepository.getLocation(vehicleId);
				float cost = this.costCache.calculateCost(vehiclePosition, waypointId);
				costSet.add(new Cost(vehicleId, cost));
			}
		}

		if (costSet.size() == 0)
		{
			String errorString = "Requested optimal car, but no cars are not-occupied..";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		// Retrieve the element with the lowest cost
		Cost leastCost = costSet.pollFirst();

		return leastCost.getVehicleId();
	}
}

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
	private CostCache costCache;

	// Cost object used to sort according to cost, but keep association with ID
	private class Cost implements Comparable<Cost>
	{
		private long vehicleId;
		private int cost;

		public Cost (long vehicleId, int cost)
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
			return this.cost - cost.cost;
		}
	}

	@Autowired
	public ResourceManager (CostCache costCache, LocationRepository locationRepository, VehicleRepository vehicleRepository)
	{
		this.log = LoggerFactory.getLogger(ResourceManager.class);

		this.log.info("Initializing ResourceManager...");
		this.locationRepository = locationRepository;
		this.vehicleRepository = vehicleRepository;
		this.costCache = costCache;
		this.log.info("Initialized ResourceManager.");
	}

	public int getNumAvailableCars()
	{
		int numAvailableCars = 0;

		for (long vehicleId: this.vehicleRepository.getVehicleIds())
		{
			if (!this.vehicleRepository.isOccupied(vehicleId))
			{
				++numAvailableCars;
			}
		}

		return numAvailableCars;
	}

	/**
	 *
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
			if (!this.vehicleRepository.isOccupied(vehicleId))
			{
				long vehiclePosition = this.locationRepository.getLocation(vehicleId);
				int cost = this.costCache.calculateCost(vehiclePosition, waypointId);
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

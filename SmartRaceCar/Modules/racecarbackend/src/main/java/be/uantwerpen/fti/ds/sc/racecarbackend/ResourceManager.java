package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.NoSuchElementException;
import java.util.TreeSet;

@Service
public class ResourceManager
{
	private Logger log;
	private NavigationManager navigationManager;
	private VehicleManager vehicleManager;
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
	public ResourceManager (CostCache costCache, NavigationManager navigationManager, VehicleManager vehicleManager)
	{
		this.log = LoggerFactory.getLogger(ResourceManager.class);

		this.log.info("Initializing ResourceManager...");
		this.navigationManager = navigationManager;
		this.vehicleManager = vehicleManager;
		this.costCache = costCache;
		this.log.info("Initialized ResourceManager.");
	}

	/**
	 *
	 * @param waypointId
	 * @return
	 */
	public long getOptimalCar (long waypointId) throws NoSuchElementException
	{
		if (this.vehicleManager.getNumVehicles() == 0)
		{
			String errorString = "Requested optimal car, but no cars exist.";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		// We use a TreeSet here to keep the cost objects
		// TreeSet's keep their values in an ordered fashion by default,
		// This helps us extract the element with the least cost in the end
		TreeSet<Cost> costSet = new TreeSet<>();

		for (long vehicleId: this.vehicleManager.getVehicleIds())
		{
			// isOccupied can potentially throw an exception if we request a non-existent vehicle
			// But this shouldn't happen since we just asked the vehicle manager for its vehicles
			// this exception will be caught higher up the call stack, since we can also throw
			// it ourselves.
			if (!this.vehicleManager.isOccupied(vehicleId))
			{
				long vehiclePosition = this.navigationManager.getLocation(vehicleId);
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

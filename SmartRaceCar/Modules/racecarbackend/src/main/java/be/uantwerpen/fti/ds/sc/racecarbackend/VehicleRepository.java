package be.uantwerpen.fti.ds.sc.racecarbackend;

import java.util.List;

public interface VehicleRepository
{
	/**
	 * Returns a list of with the ID of every vehicle.
	 * @return
	 */
	public List<Long> getVehicleIds();

	/**
	 * Return the number of existing vehicles.
	 * @return
	 */
	public int getNumVehicles();
}

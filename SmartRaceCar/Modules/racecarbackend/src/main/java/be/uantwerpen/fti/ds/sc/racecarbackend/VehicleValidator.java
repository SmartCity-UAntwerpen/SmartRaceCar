package be.uantwerpen.fti.ds.sc.racecarbackend;

public interface VehicleValidator
{
	/**
	 * Checks whether or not a vehicle exists.
	 *
	 * @param vehicleId The id of the vehicle to be checked
	 * @return  True if the vehicle exists, false if it doesn't
	 */
	boolean exists(long vehicleId);
}

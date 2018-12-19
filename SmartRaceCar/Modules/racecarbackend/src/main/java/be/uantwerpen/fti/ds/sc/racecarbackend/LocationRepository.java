package be.uantwerpen.fti.ds.sc.racecarbackend;

public interface LocationRepository
{
	/**
	 * Get the location of the vehicle with ID vehicleId.
	 * If the vehicle doesn't exist an IndexOutOfBoundsExecption is thrown.
	 * @param vehicleId
	 * @return
	 */
	public long getLocation(long vehicleId);

	/**
	 * Set the location of a vehicle.
	 * @param vehicleId     The vehicle whose location we want to change.
	 * @param locationId    The new location of the vehicle.
	 */
	public void setLocation(long vehicleId, long locationId);
}

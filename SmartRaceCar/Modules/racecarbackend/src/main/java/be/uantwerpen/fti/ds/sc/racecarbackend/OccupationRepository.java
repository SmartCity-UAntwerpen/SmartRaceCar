package be.uantwerpen.fti.ds.sc.racecarbackend;

public interface OccupationRepository
{
	/**
	 * Check if the vehicle with the given ID is occupied.
	 * @param vehicleId
	 * @return
	 */
	public boolean isOccupied(long vehicleId);
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

/**
 * Model that describes a job request.
 */
class Job
{
	private long startId; // ID of the start waypoint of the route.
	private long endId; // ID of the end waypoint of the route.
	private long vehicleId; // ID of the vehicle.
	private int progress;

	/**
	 * Create a job object.
	 * Progress is initialized to 0 and stored as a percentage.
	 *
	 * @param jobId     ID of the job
	 * @param startId   ID of the start waypoint of the route
	 * @param endId     ID of the end waypoint of the route
	 * @param vehicleId ID of the vehicle
	 */
	public Job(long startId, long endId, long vehicleId)
	{
		this.startId = startId;
		this.endId = endId;
		this.vehicleId = vehicleId;
		this.progress = 0;
	}

	/**
	 * Method to get the starting waypoint's ID.
	 *
	 * @return Long of the starting waypoint's ID
	 */
	public long getStartId()
	{
		return startId;
	}

	/**
	 * Method to get the ending waypoint's ID.
	 *
	 * @return Long of the ending waypoint's ID
	 */
	public long getEndId()
	{
		return endId;
	}

	/**
	 * Method to get the vehicle's ID
	 *
	 * @return Long of the vehicle's ID
	 */
	public long getVehicleId()
	{
		return vehicleId;
	}

	public int getProgress()
	{
		return this.progress;
	}

	public void setProgress(int newProgress)
	{
		this.progress = newProgress;
	}
}
package be.uantwerpen.fti.ds.sc.racecarbackend.jobs;

/**
 * Model that describes a job request.
 */
class Job
{
	private long jobId;     // ID of the job
	private long startId; 	// ID of the start waypoint of the route.
	private long endId; 	// ID of the end waypoint of the route.
	private long vehicleId; // ID of the vehicle.
	private int progress;
	private boolean backboneNotified;	// Whether or not the backbone has been notified about us "almost" completing our job.
	private boolean alert; // Whether or not the job has been alerted or is just an execution

	/**
	 * Create a job object.
	 * Progress is initialized to 0 and stored as a percentage.
	 *
	 * @param jobId     ID of the job
	 * @param startId   ID of the start waypoint of the route
	 * @param endId     ID of the end waypoint of the route
	 * @param vehicleId ID of the vehicle
	 */
	public Job(long jobId, long startId, long endId, long vehicleId)
	{
		this.jobId = jobId;
		this.startId = startId;
		this.endId = endId;
		this.vehicleId = vehicleId;
		this.progress = 0;
		this.backboneNotified = false;
		this.alert = false;
	}

	public long getJobId()
	{
		return this.jobId;
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
	public void setStartId(long startId)
	{
		 this.startId = startId;
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

	public void setVehicleId(long vehicleId)
	{
		this.vehicleId = vehicleId;
	}

	/**
	 * Return the progress of this job (As a percentage)
	 * @return
	 */
	public int getProgress()
	{
		return this.progress;
	}

	/**
	 * Set the progress of this job (Should be a percentage)
	 * @param newProgress
	 */
	public void setProgress(int newProgress)
	{
		this.progress = newProgress;
	}

	public void setBackboneNotified(boolean backboneNotified)
	{
		this.backboneNotified = backboneNotified;
	}

	public boolean isBackboneNotified()
	{
		return this.backboneNotified;
	}
	public boolean getAlert()
	{
		return this.alert;
	}
	public void setAlert(Boolean preparation)
	{
		this.alert= preparation;
	}


}
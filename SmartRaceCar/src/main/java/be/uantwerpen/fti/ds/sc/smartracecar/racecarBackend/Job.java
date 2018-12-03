package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

/**
 * Model that describes a job request.
 */
class Job
{

	private Long idJob; //ID of the job.
	private Long idStart; // ID of the start waypoint of the route.
	private Long idEnd; // ID of the end waypoint of the route.
	private Long idVehicle; // ID of the vehicle.

	/**
	 * Model that describes a job request.
	 *
	 * @param idJob     ID of the job
	 * @param idStart   ID of the start waypoint of the route
	 * @param idEnd     ID of the end waypoint of the route
	 * @param idVehicle ID of the vehicle
	 */
	Job(Long idJob, Long idStart, Long idEnd, Long idVehicle)
	{
		this.idJob = idJob;
		this.idStart = idStart;
		this.idEnd = idEnd;
		this.idVehicle = idVehicle;
	}

	/**
	 * Method to get the starting waypoint's ID.
	 *
	 * @return Long of the starting waypoint's ID
	 */
	Long getIdStart()
	{
		return idStart;
	}

	/**
	 * Method to get the ending waypoint's ID.
	 *
	 * @return Long of the ending waypoint's ID
	 */
	Long getIdEnd()
	{
		return idEnd;
	}

	/**
	 * Method to get the vehicle's ID
	 *
	 * @return Long of the vehicle's ID
	 */
	Long getIdVehicle()
	{
		return idVehicle;
	}

	public Long getIdJob()
	{
		return idJob;
	}
}
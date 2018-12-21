package be.uantwerpen.fti.ds.sc.core;

/**
 * Drive parameters store object.
 */
public class Drive
{

	private float steer; //Rotation of the wheels.
	private float throttle; //speed of the vehicle's wheels.

	/**
	 * Drive parameters store object.
	 *
	 * @param steer    rotation value for the vehicle wheels.
	 * @param throttle throttle value for the vehicle wheels.
	 */
	public Drive(float steer, float throttle)
	{
		this.steer = steer;
		this.throttle = throttle;
	}
}
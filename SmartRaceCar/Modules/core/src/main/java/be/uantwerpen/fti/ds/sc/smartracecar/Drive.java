package be.uantwerpen.fti.ds.sc.smartracecar;

/**
 * Drive parameters store object.
 */
class Drive
{

	private float steer; //Rotation of the wheels.
	private float throttle; //speed of the vehicle's wheels.

	/**
	 * Drive parameters store object.
	 *
	 * @param steer    rotation value for the vehicle wheels.
	 * @param throttle throttle value for the vehicle wheels.
	 */
	Drive(float steer, float throttle)
	{
		this.steer = steer;
		this.throttle = throttle;
	}
}
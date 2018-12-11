package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;

import java.util.Arrays;

/**
 * Model that describes a simulated vehicle.
 */
class SimulatedVehicle
{

	private long startPoint = -1; // Point at where the vehicle is started. Set at -1 by default to indicate it hasn't been set yet.
	private String name; // Unused parameter to set the vehicle name.
	private float speed = 0; // Unused parameter to set the vehicle speed.
	private boolean deployed = false; // To indicate if the simulation has been started(run) yet.
	private boolean available = false; // To indicate if the vehicle is available for jobs and request (if it is stopped or not)
	private Simulation simulatedCore; // Simulated Core-module.
	private Simulation simulatedSimKernel; // Simulated RosKernel(now being SimKernel)-module.
	private int listeningPort; // Port on which the Core-module will be listening to for messages over TCP sockets.

	/**
	 * Model that describes a simulated vehicle.
	 *
	 * @param simulationID The ID the vehicle will be receiving.
	 * @param jarPath      Path where the jar files to be simulated can be found.
	 */
	SimulatedVehicle(long simulationID, String jarPath, boolean showSimulatedOutput)
	{
		name = "SimCar" + simulationID;
		this.simulatedCore = new Simulation(jarPath + "Core.jar", showSimulatedOutput);
		this.simulatedSimKernel = new Simulation(jarPath + "SimKernel.jar", showSimulatedOutput);
	}

	/**
	 * Method to start(run) the simulation. Will request the deployment of the simulated jars by
	 * starting their threaded simulation. Sets the deployment(run) and available(stopped or not) parameters.
	 *
	 * @param portOne The TCP socket port on which the Core will be listening on and the Simkernel will be sending to.
	 * @param portTwo The TCP socket port on which the SimKernel will be listening on and the Core will be sending to.
	 */
	void start(int portOne, int portTwo)
	{
		this.listeningPort = portOne;
		simulatedCore.start(Arrays.asList(Long.toString(startPoint), Integer.toString(portOne), Integer.toString(portTwo)));
		simulatedSimKernel.start(Arrays.asList(Integer.toString(portTwo), Integer.toString(portOne)));
		this.deployed = true;
		this.available = true;
	}

	/**
	 * Method to stop the simulation(pausing it in reality). It simply disables the vehicle from receiving
	 * any jobs or requests and makes it hidden from the visualization. This allows it to be altered or restarted.
	 * Sends TCP message to the Core to make it do it's methods to stop the vehicle.
	 */
	void stop()
	{
		TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("stop"), listeningPort);
		this.available = false;
	}

	/**
	 * Method to kill the simulation.
	 * Sends TCP message to the Core to make it do it's methods to kill the vehicle in a clean way.
	 */
	void kill()
	{
		TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("kill"), listeningPort);
		simulatedCore.stop();
		simulatedSimKernel.stop();
	}

	/**
	 * Method to restart the simulation. Makes the simulation start on the startpoint again and makes the vehicle
	 * available again.
	 * Sends TCP message to the Core to make it do it's methods to restart the vehicle.
	 */
	void restart()
	{
		TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("restart"), listeningPort);
		this.available = true;
	}

	/**
	 * Method for when the simulation is stopped(paused) but needs to be started(run) again. This simply makes the
	 * vehicle available again by sending the Core a TCP message.
	 */
	void run()
	{
		TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("start"), listeningPort);
		this.available = true;
	}

	/**
	 * Method for when the startpoint needs to be altered.
	 * Sends TCP message to the Core to make it do it's methods to set the startpoint again.
	 */
	void setStartPoint()
	{
		TCPUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startpoint", startPoint), listeningPort);
	}

	/**
	 * Method to get the current set startpoint.
	 *
	 * @return Returns the currently set startpoint.
	 */
	long getStartPoint()
	{
		return startPoint;
	}

	/**
	 * Method to set a new name for the simulated vehicle.
	 *
	 * @param name String containing the new name for the vehicle.
	 */
	void setName(String name)
	{
		this.name = name;
	}

	/**
	 * Method to set a new speed for the simulated vehicle.
	 *
	 * @param speed Float containing the new speed for the vehicle.
	 */
	void setSpeed(float speed)
	{
		this.speed = speed;
	}

	/**
	 * Method to set a new startpoint for the simulated vehicle.
	 *
	 * @param startPoint Long containing the new waypoint ID for the vehicle.
	 */
	void setStartPoint(long startPoint)
	{
		this.startPoint = startPoint;
	}

	/**
	 * Method to know if the vehicle is deployed(simulation started) or not.
	 *
	 * @return Status of deployment of simulation.
	 */
	boolean isDeployed()
	{
		return deployed;
	}

	/**
	 * Method to know if the vehicle is available for jobs/requests and if it will show up on the visualization.
	 *
	 * @return Status of availability.
	 */
	boolean isAvailable()
	{
		return available;
	}
}

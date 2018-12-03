package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import java.util.List;

/**
 * Model that describes a simulation of a Jar.
 */
class Simulation
{

	private String location; // Location of the jar file
	private boolean running; // Parameter to know if it's already running
	private boolean showSimulatedOutput; // boolean that controls whether or not the simulated output is shown
	private Thread thread; // Thread containing the simulation
	private List<String> runArguments; // All arguments used to star the jar.

	/**
	 * Model that describes a simulation of a Jar.
	 *
	 * @param location location of the jar.
	 */
	Simulation(String location, boolean showSimulatedOutput)
	{
		this.location = location;
		this.running = false;
		this.thread = null;
		this.showSimulatedOutput = showSimulatedOutput;
	}

	/**
	 * Starts the threaded process of running the jar in a process.
	 *
	 * @param runArguments All input arguments that the jar needs.
	 * @return boolean if the start was successful.
	 */
	boolean start(List<String> runArguments)
	{
		this.runArguments = runArguments;
		if (!running)
		{
			thread = new Thread(new CoreProcess());
			thread.start();
			running = true;
			return true;
		} else
		{
			return false;
		}
	}

	/**
	 * Interrupts the thread containing the running jar.
	 *
	 * @return Boolean if the stop was successful.
	 */
	boolean stop()
	{
		if (running)
		{
			thread.interrupt();
			return true;
		} else
		{
			return false;
		}
	}

	/**
	 * process that will contain the jar. Uses a processbuilder.
	 */
	private class CoreProcess implements Runnable
	{
		@Override
		public void run()
		{
			//Create process
			ProcessBuilder processBuilder = new ProcessBuilder("java");
			if (showSimulatedOutput)
			{
				processBuilder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
				processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT);
			}
			List<String> processCommands = processBuilder.command();
			processCommands.add("-jar");
			processCommands.add(location);
			processCommands.addAll(runArguments);
			processBuilder.command(processCommands);
			try
			{
				Process process = processBuilder.start();
			} catch (Exception e)
			{
				e.printStackTrace();
				running = false;
				return;
			}
		}
	}
}

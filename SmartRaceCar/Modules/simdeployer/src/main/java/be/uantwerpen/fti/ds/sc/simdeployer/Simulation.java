package be.uantwerpen.fti.ds.sc.simdeployer;

import java.io.IOException;
import java.util.List;

public interface Simulation
{
	/**
	 * Launch a simulation with the given command line arguments.
	 * This should be done in a non-blocking fashion.
	 * @param args          Command line arguments passed to the VM
	 * @throws IOException  Exceptions can be thrown when starting the VM.
	 * @return The return value from the VM start command.
	 */
	public void run(List<String> args) throws IOException;

	/**
	 * Stop the Simulation and return the process' return value.
	 * @return
	 */
	public int stop() throws IOException, InterruptedException;
}

package be.uantwerpen.fti.ds.sc.simdeployer;

import java.io.IOException;
import java.util.List;

public interface VirtualMachine
{
	/**
	 * Launch a virtual machine with the given command line arguments.
	 * VMs should be launched in a non-blocking fashion. For Docker containers,
	 * this is done through the "-d" (detached) option.
	 * @param args          Command line arguments passed to the VM
	 * @throws IOException  Exceptions can be thrown when starting the VM.
	 * @return The return value from the VM start command.
	 */
	public void run(List<String> args) throws IOException;

	/**
	 * Stop the Virtual Machine and return the process' return value.
	 * @return
	 */
	public int stop() throws IOException, InterruptedException;
}

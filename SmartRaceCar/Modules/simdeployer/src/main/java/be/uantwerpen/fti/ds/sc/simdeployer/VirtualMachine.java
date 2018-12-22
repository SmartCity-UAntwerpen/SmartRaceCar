package be.uantwerpen.fti.ds.sc.simdeployer;

import java.io.IOException;
import java.util.List;

public interface VirtualMachine
{
	/**
	 * Launch a virtual machine with the given command line arguments.
	 * @param args          Command line arguments passed to the VM
	 * @throws IOException  Exceptions can be thrown when starting the VM.
	 */
	public void run(List<String> args) throws Exception;

	/**
	 * Stop the Virtual Machine and return the process' return value.
	 * @return
	 */
	public int stop();
}

package be.uantwerpen.fti.ds.sc.simdeployer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.*;

public class HyperVisor
{
	private static final int SIMULATION_PORT_1 = 1024;
	private static final int SIMULATION_PORT_2 = 1025;

	private Logger log;
	private SimDeployerParameters simDeployerParameters;
	private Map<Long, VirtualMachine> virtualMachines;

	public HyperVisor (SimDeployerParameters simDeployerParameters)
	{
		this.log = LoggerFactory.getLogger(HyperVisor.class);
		this.simDeployerParameters = simDeployerParameters;
		this.virtualMachines = new HashMap<>();
	}

	public void launch(long simulationId, long startpoint) throws Exception
	{
		VirtualMachineFactory factory = new VirtualMachineFactory();
		VirtualMachine vm = factory.createDockerContainer(simulationId, this.simDeployerParameters.getDockerImage());
		this.virtualMachines.put(simulationId, vm);

		List<String> arguments = new ArrayList<>();
		arguments.add(Long.toString(startpoint));
		arguments.add(Integer.toString(SIMULATION_PORT_1));
		arguments.add(Integer.toString(SIMULATION_PORT_2));
		arguments.add(Long.toString(simulationId));

		try
		{
			vm.run(arguments);
		}
		catch (Exception e)
		{
			// Catch, log and rethrow
			this.log.error("Failed to run Virtual Machine.", e);
			throw e;
		}
	}

	public void stop(long simulationId) throws NoSuchElementException
	{
		if (!this.virtualMachines.containsKey(simulationId))
		{
			String errorString = "Tried to stop simulation " + simulationId + ", but HyperVisor doesn't have this simulation.";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		this.virtualMachines.get(simulationId).stop();
		this.virtualMachines.remove(simulationId);
	}
}

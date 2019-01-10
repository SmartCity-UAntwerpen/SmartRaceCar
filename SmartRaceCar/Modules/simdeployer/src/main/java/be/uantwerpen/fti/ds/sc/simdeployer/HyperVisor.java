package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.DockerAspect;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.naming.InvalidNameException;
import java.io.IOException;
import java.util.*;

public class HyperVisor
{
	private Logger log;
	private Configuration configuration;
	private Map<Long, Simulation> virtualMachines;

	public HyperVisor (Configuration configuration)
	{
		this.log = LoggerFactory.getLogger(HyperVisor.class);
		this.configuration = configuration;
		this.virtualMachines = new HashMap<>();
	}

	public void launch(long simulationId, long startpoint) throws IOException, InvalidNameException
	{
		DockerAspect dockerAspect = (DockerAspect) configuration.get(AspectType.DOCKER);
		SimulationFactory factory = new SimulationFactory(configuration);
		Simulation vm = null;

		try
		{
			vm = factory.createDockerContainer(simulationId, dockerAspect.getImageName());
		}
		catch (InvalidNameException ine)
		{
			String errorString = "Failed to create Virtual Machine.";
			this.log.error(errorString, ine);
			throw ine;
		}

		this.virtualMachines.put(simulationId, vm);

		List<String> arguments = new ArrayList<>();
		arguments.add(Long.toString(startpoint));
		arguments.add(Long.toString(simulationId));
		arguments.add(dockerAspect.getContainerVolume());

		try
		{
			vm.run(arguments);
		}
		catch (IOException ie)
		{
			// Catch, log and rethrow
			this.log.error("Failed to run Virtual Machine.", ie);
			throw ie;
		}
	}

	public void stop(long simulationId) throws NoSuchElementException, IOException, InterruptedException, SimulationException
	{
		if (!this.virtualMachines.containsKey(simulationId))
		{
			String errorString = "Tried to stop simulation " + simulationId + ", but HyperVisor doesn't have this simulation.";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		try
		{
			int returnValue = this.virtualMachines.get(simulationId).stop();

			if (returnValue != 0)
			{
				String errorString = "Failed to stop Virtual Machine, process returned " + returnValue;
				this.log.error(errorString);
				throw new SimulationException(errorString);
			}

			this.virtualMachines.remove(simulationId);
		}
		catch (IOException | InterruptedException ie)
		{
			// Catch, log and rethrow
			this.log.error("Failed to stop Virtual Machine.", ie);
			throw ie;
		}
	}
}

package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

import javax.naming.InvalidNameException;

public class VirtualMachineFactory
{
	public VirtualMachineFactory()
	{
	}

	public VirtualMachine createDockerContainer(long simulationId, String name) throws InvalidNameException
	{
		return new Container(simulationId, name, "Docker-Container-" + simulationId);
	}
}

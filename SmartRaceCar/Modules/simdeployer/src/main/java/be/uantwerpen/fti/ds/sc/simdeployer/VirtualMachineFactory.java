package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.DockerAspect;
import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

import javax.naming.InvalidNameException;

public class VirtualMachineFactory
{
	private Configuration configuration;

	public VirtualMachineFactory(Configuration configuration)
	{
		this.configuration = configuration;
	}

	public VirtualMachine createDockerContainer(long simulationId, String name) throws InvalidNameException
	{
		return new Container(this.configuration, simulationId, name, "Docker-Container-" + simulationId);
	}
}

package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

import javax.naming.InvalidNameException;

public class VirtualMachineFactory
{
	private SimDeployerParameters parameters;

	public VirtualMachineFactory(SimDeployerParameters parameters)
	{
		this.parameters = parameters;
	}

	public VirtualMachine createDockerContainer(long simulationId, String name) throws InvalidNameException
	{
		return new Container(this.parameters, simulationId, name, "Docker-Container-" + simulationId);
	}
}

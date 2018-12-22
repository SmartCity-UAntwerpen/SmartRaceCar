package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

public class VirtualMachineFactory
{
	public VirtualMachine createDockerContainer(long simulationId, String name)
	{
		return new Container(simulationId, name);
	}
}

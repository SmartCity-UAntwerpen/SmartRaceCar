package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

public class VirtualMachineFactory
{
	public VirtualMachine createDockerContainer(String name)
	{
		return new Container(name);
	}
}

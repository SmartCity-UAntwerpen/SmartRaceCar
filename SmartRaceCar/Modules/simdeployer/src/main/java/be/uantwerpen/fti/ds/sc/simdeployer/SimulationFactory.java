package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.simdeployer.docker.Container;

import javax.naming.InvalidNameException;

public class SimulationFactory
{
	private Configuration configuration;

	public SimulationFactory(Configuration configuration)
	{
		this.configuration = configuration;
	}

	public Simulation createDockerContainer(long simulationId, String name) throws InvalidNameException
	{
		return new Container(this.configuration, simulationId, name, "Docker-Container-" + simulationId);
	}
}

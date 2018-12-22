package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import be.uantwerpen.fti.ds.sc.simdeployer.VirtualMachine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Container implements VirtualMachine
{
	private static final String DOCKER_COMMAND = "docker";
	private static final String DOCKER_RUN = "run";

	private Logger log;
	private long simulationId;
	private String name;
	private Process dockerProcess;

	public Container (long simulationId, String name)
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.log.info("Created Docker Container " + name + ", with Simulation ID " + simulationId);
		this.simulationId = simulationId;
		this.name = name;
	}

	@Override
	public void run(List<String> args) throws IOException
	{
		List<String> commandLine = new ArrayList<>();
		commandLine.add(DOCKER_COMMAND);
		commandLine.add(DOCKER_RUN);
		commandLine.add(this.name);
		commandLine.addAll(args);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);
		processBuilder = processBuilder.redirectOutput(new File("Docker-Simulation-" + this.simulationId + "-" + this.name + "-stdout.log"));

		this.log.info("Running Docker Container " + this.name + ", with Simulation ID " + this.simulationId);

		try
		{
			this.dockerProcess = processBuilder.start();
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to start Docker process.", ioe);
			throw ioe;
		}
	}

	@Override
	public int stop()
	{
		this.log.info("Stopping Docker Container " + this.name + ", with Simulation ID " + this.simulationId);
		this.dockerProcess.destroy();
		return this.dockerProcess.exitValue();
	}
}

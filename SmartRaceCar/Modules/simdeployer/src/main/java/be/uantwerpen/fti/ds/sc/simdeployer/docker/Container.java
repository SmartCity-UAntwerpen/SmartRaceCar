package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import be.uantwerpen.fti.ds.sc.simdeployer.VirtualMachine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Container implements VirtualMachine
{
	private static final String DOCKER_COMMAND = "docker";
	private static final String DOCKER_RUN = "run";

	private Logger log;
	private String name;
	private Process dockerProcess;

	public Container (String name)
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.name = name;
	}

	@Override
	public void run(List<String> args) throws IOException
	{
		List<String> commandLine = new ArrayList<>();
		commandLine.add(DOCKER_COMMAND);
		commandLine.add(DOCKER_RUN);
		commandLine.add(this.name);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine).inheritIO();

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

	public void stop()
	{
		this.dockerProcess.destroy();
	}
}

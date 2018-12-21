package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Container
{
	private static final String DOCKER_COMMAND = "docker";
	private static final String DOCKER_RUN = "run";

	private Logger log;
	private String name;

	public Container (String name)
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.name = name;
	}

	public void run() throws IOException
	{
		List<String> commandLine = new ArrayList<>();
		commandLine.add(DOCKER_COMMAND);
		commandLine.add(DOCKER_RUN);
		commandLine.add(this.name);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine).inheritIO();

		try
		{
			Process process = processBuilder.start();
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to start Docker process.", ioe);
			throw ioe;
		}
	}
}

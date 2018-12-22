package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.Command;
import be.uantwerpen.fti.ds.sc.common.CommandParser;
import be.uantwerpen.fti.ds.sc.common.TCPListener;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.*;

public class SimDeployerV2 implements TCPListener
{
	private static final String ACK = "ACK";
	private static final String NACK = "NACK";
	private static final String PONG = "PONG";

	private static final String STARTPOINT_KEY = "startpoint";
	private static final String SPEED_KEY = "speed";
	private static final String NAME_KEY = "name";

	private Logger log;
	private TCPUtils simulationFrontend;
	private Map<Long, Long> startPoints;
	private HyperVisor hyperVisor;

	private String createSimulation(long simulationId)
	{
		if (this.startPoints.containsKey(simulationId))
		{
			this.log.error("Tried to add simulation with existing ID (" + simulationId + ").");
			return NACK;
		}

		this.startPoints.put(simulationId, -1L);
		return ACK;
	}

	private String runSimulation(long simulationId)
	{
		if (!this.startPoints.containsKey(simulationId))
		{
			this.log.error("Tried to start simulation that doesn't exist.");
			return NACK;
		}

		if (this.startPoints.get(simulationId) == -1L)
		{
			this.log.error("Tried to start simulation that doesn't have a startpoint.");
			return NACK;
		}

		try
		{
			this.hyperVisor.launch(simulationId);
		}
		catch (IOException ioe)
		{
			this.log.error("An error occurred while trying to launch a container.", ioe);
			return NACK;
		}

		return ACK;
	}

	private String set(long simulationId, String key, String value)
	{
		if (key.equalsIgnoreCase(STARTPOINT_KEY))
		{
			this.startPoints.put(simulationId, Long.parseLong(value));
			return ACK;
		}
		else if (key.equalsIgnoreCase(SPEED_KEY))
		{
			this.log.warn("Got set command for speed, this command is not supported, ignoring.");
			return NACK;
		}
		else if (key.equalsIgnoreCase(NAME_KEY))
		{
			this.log.warn("Got set command for name, this command is not supported, ignoring.");
			return NACK;
		}

		this.log.error("Got unsupported set command \"set " + key + " " + value + "\"");

		return NACK;
	}

	private String stopSimulation(long simulationId)
	{
		if (!this.startPoints.containsKey(simulationId))
		{
			String errorString = "Tried to stop non-existent simulation (" + simulationId + ").";
			this.log.error(errorString);
			return NACK;
		}

		try
		{
			this.hyperVisor.stop(simulationId);
		}
		catch (NoSuchElementException nsee)
		{
			String errorString = "An exception was thrown while trying to stop simulation " + simulationId + ".";
			this.log.error(errorString, nsee);
			return NACK;
		}

		return ACK;
	}

	private String killSimulation(long simulationId)
	{
		if (!this.startPoints.containsKey(simulationId))
		{
			String errorString = "Tried to kill non-existent simulation (" + simulationId + ").";
			this.log.error(errorString);
			return NACK;
		}

		this.startPoints.remove(simulationId);

		return ACK;
	}

	private String executeCommand(Command command, String payload)
	{
		switch (command)
		{
			case CREATE:
			{
				long simulationId = Long.parseLong(payload);
				return this.createSimulation(simulationId);
			}

			case RUN:
			{
				long simulationId = Long.parseLong(payload);
				return this.runSimulation(simulationId);
			}

			case STOP:
			{
				long simulationId = Long.parseLong(payload);
				return this.stopSimulation(simulationId);
			}

			case KILL:
			{
				long simulationId = Long.parseLong(payload);
				return this.killSimulation(simulationId);
			}

			// The effect of a restart is the same as that of a "run",
			// This is something that needs to be cleaned up in the simulation front end,
			// Nothing we can do about this, I'm afraid
			case RESTART:
			{
				long simulationId = Long.parseLong(payload);
				return this.runSimulation(simulationId);
			}

			case SET:
			{
				String[] split = payload.split("\\s");
				long simulationId = Long.parseLong(split[0]);
				String key = split[1];
				String value = split[2];
				return this.set(simulationId, key, value);
			}

			case PING:
				return PONG;
		}

		return NACK;
	}

	public SimDeployerV2(SimDeployerParameters parameters) throws IOException
	{
		this.log = LoggerFactory.getLogger(SimDeployerV2.class);
		this.simulationFrontend = new TCPUtils(parameters.getServerPort(), this);
		this.simulationFrontend.start();
		this.startPoints = new HashMap<>();
		this.hyperVisor = new HyperVisor(parameters);
	}

	@Override
	public String parseTCP(String message)
	{
		// Split off the first part of the message, this is the command, the rest is payload
		String[] parts = message.split("\\s");
		StringBuilder payloadBuilder = new StringBuilder();

		for (int i = 1; i < parts.length; ++i)
		{
			payloadBuilder.append(parts[i]);
		}

		// Parse the command
		CommandParser commandParser = new CommandParser();
		Command command = commandParser.parseCommand(parts[0]);

		// Retrieve the payload
		String payload = payloadBuilder.toString();
		return this.executeCommand(command, payload);
	}

	public static void main(String[] args)
	{
		try
		{
			final SimDeployerV2 simDeployerV2 = new SimDeployerV2(new SimDeployerParameters(9999, "ubuntu:14.04"));
		}
		catch (IOException ioe)
		{
			System.out.println("Failed to start SimDeployer: " + ioe.getMessage());
			ioe.printStackTrace();
			System.exit(-1);
		}
	}
}

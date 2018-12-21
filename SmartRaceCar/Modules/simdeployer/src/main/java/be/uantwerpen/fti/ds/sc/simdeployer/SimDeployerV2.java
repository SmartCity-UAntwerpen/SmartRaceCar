package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.TCPListener;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class SimDeployerV2 implements TCPListener
{
	private static final String ACK = "ACK";
	private static final String NACK = "NACK";
	private static final String PONG = "PONG";

	private static final String STARTPOINT_KEY = "startpoint";
	private static final String SPEED_KEY = "speed";
	private static final String NAME_KEY = "name";

	private Logger log;
	private SimDeployerParameters parameters;
	private TCPUtils simulationFrontend;
	private List<Long> simulationIds;
	private Map<Long, Long> startPoints;

	private String createSimulation(long simulationId)
	{
		if (this.simulationIds.contains(simulationId))
		{
			this.log.error("Tried to add simulation with existing ID (" + simulationId + ").");
			return NACK;
		}

		this.simulationIds.add(simulationId);
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
				break;

			case STOP:
				break;

			case KILL:
				break;

			case RESTART:
				break;

			case SET:
			{
				String[] split = payload.split("\\s");
				long simulationId = Long.parseLong(split[0]);
				String key = split[1];
				String value = split[2];
				this.set(simulationId, key, value);

				break;
			}

			case PING:
				return PONG;
		}

		return NACK;
	}

	public SimDeployerV2(SimDeployerParameters parameters) throws IOException
	{
		this.log = LoggerFactory.getLogger(SimDeployerV2.class);
		this.parameters = parameters;
		this.simulationFrontend = new TCPUtils(parameters.getServerPort(), this);
		this.simulationFrontend.start();
		this.simulationIds = new ArrayList<>();
	}

	@Override
	public String parseTCP(String message) throws IOException
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
		Command command = commandParser.parse(parts[0]);

		// Retrieve the payload
		String payload = payloadBuilder.toString();
		return this.executeCommand(command, payload);
	}
}

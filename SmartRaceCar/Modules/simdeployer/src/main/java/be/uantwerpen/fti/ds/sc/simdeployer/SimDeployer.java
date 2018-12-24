package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.commands.*;
import be.uantwerpen.fti.ds.sc.common.TCPListener;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.naming.InvalidNameException;
import java.io.IOException;
import java.util.*;

public class SimDeployer implements TCPListener
{
	private static final String ACK = "ACK";
	private static final String NACK = "NACK";
	private static final String PONG = "PONG";
	private static final String DEFAULT_CONFIG_FILE = "./SimDeployer.properties";

	private Logger log;
	private TCPUtils simulationFrontend;
	private Map<Long, Long> startPoints;
	private HyperVisor hyperVisor;

	private String createSimulation(Range simulationIds)
	{
		for (long simulationId: simulationIds.generate())
		{
			if (this.startPoints.containsKey(simulationId))
			{
				this.log.error("Tried to add simulation with existing ID (" + simulationId + ").");
				return NACK;
			}

			this.startPoints.put(simulationId, -1L);
		}

		return ACK;
	}

	private String runSimulation(Range simulationIds)
	{
		for (long simulationId: simulationIds.generate())
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
				this.hyperVisor.launch(simulationId, this.startPoints.get(simulationId));
			}
			catch (IOException | InvalidNameException ie)
			{
				this.log.error("An error occurred while trying to launch a virtual machine.", ie);
				return NACK;
			}
		}
		return ACK;
	}

	private String set(SetCommand setCommand)
	{
		if (setCommand.getKey() == SetParameter.STARTPOINT)
		{
			for (long simulationId: setCommand.getSimulationId().generate())
			{
				this.startPoints.put(simulationId, Long.parseLong(setCommand.getValue()));
			}

			return ACK;
		}
		else if (setCommand.getKey() == SetParameter.SPEED)
		{
			this.log.warn("Got set command for speed, this command is not supported, ignoring.");
			return NACK;
		}
		else if (setCommand.getKey() == SetParameter.NAME)
		{
			this.log.warn("Got set command for name, this command is not supported, ignoring.");
			return NACK;
		}

		this.log.error("Got unsupported set command \"" + setCommand + "\"");
		return NACK;
	}

	private String stopSimulation(Range simulationIds)
	{
		for (long simulationId: simulationIds.generate())
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
			catch (NoSuchElementException | IOException | InterruptedException | VirtualMachineException e)
			{
				String errorString = "An exception was thrown while trying to stop virtual machine " + simulationId + ".";
				this.log.error(errorString, e);
				return NACK;
			}
		}

		return ACK;
	}

	private String killSimulation(Range simulationIds)
	{
		if (this.stopSimulation(simulationIds).equalsIgnoreCase(NACK))
		{
			return NACK;
		}

		for (long simulationId: simulationIds.generate())
		{
			if (!this.startPoints.containsKey(simulationId))
			{
				String errorString = "Tried to kill non-existent simulation (" + simulationId + ").";
				this.log.error(errorString);
				return NACK;
			}

			this.startPoints.remove(simulationId);
		}

		return ACK;
	}

	private String executeCommand(Command command)
	{
		switch (command.getCommandType())
		{
			case CREATE:
			{
				VehicleCommand createCommand = (VehicleCommand) command;
				return this.createSimulation(createCommand.getSimulationId());
			}

			case RUN:
			{
				VehicleCommand runCommand = (VehicleCommand) command;
				return this.runSimulation(runCommand.getSimulationId());
			}

			case STOP:
			{
				VehicleCommand stopCommand = (VehicleCommand) command;
				return this.stopSimulation(stopCommand.getSimulationId());
			}

			// Since kill removes a simulation ID from tracking, it also implies stopping the simulation gracefully first.
			case KILL:
			{
				VehicleCommand killCommand = (VehicleCommand) command;
				return this.killSimulation(killCommand.getSimulationId());
			}

			// The effect of a restart is the same as that of a "run",
			// This is something that needs to be cleaned up in the simulation front end,
			// Nothing we can do about this, I'm afraid...
			case RESTART:
			{
				VehicleCommand restartCommand = (VehicleCommand) command;
				return this.runSimulation(restartCommand.getSimulationId());
			}

			case SET:
			{
				SetCommand setCommand = (SetCommand) command;
				return this.set(setCommand);
			}

			case PING:
				return PONG;
		}

		return NACK;
	}

	public SimDeployer(SimDeployerParameters parameters) throws IOException
	{
		this.log = LoggerFactory.getLogger(SimDeployer.class);
		this.simulationFrontend = new TCPUtils(parameters.getServerPort(), this);
		this.simulationFrontend.start();
		this.startPoints = new HashMap<>();
		this.hyperVisor = new HyperVisor(parameters);
	}

	@Override
	public String parseTCP(String message)
	{
		CommandParser parser = new CommandParser();

		try
		{
			Command command = parser.parseCommand(message);
			return this.executeCommand(command);
		}
		catch (IllegalArgumentException iae)
		{
			this.log.error("Failed to parse command \"" + message + "\"", iae);
			return NACK;
		}
	}

	public static void main(String[] args)
	{
		SimDeployerParameterParser parameterParser = new SimDeployerParameterParser();

		try
		{
			final SimDeployer simDeployer = new SimDeployer(parameterParser.parse(DEFAULT_CONFIG_FILE));
		}
		catch (IOException ioe)
		{
			System.out.println("Failed to start SimDeployer: " + ioe.getMessage());
			ioe.printStackTrace();
			System.exit(-1);
		}
	}
}

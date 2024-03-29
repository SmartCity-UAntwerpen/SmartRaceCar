package be.uantwerpen.fti.ds.sc.common.commands;

public class CommandParser
{
	private SetParameter findSetParameter(String setParameter) throws IllegalArgumentException
	{
		if (setParameter.equals(SetParameter.STARTPOINT.toString()))
		{
			return SetParameter.STARTPOINT;
		}
		else if (setParameter.equals(SetParameter.SPEED.toString()))
		{
			return SetParameter.SPEED;
		}
		else if (setParameter.equals(SetParameter.NAME.toString()))
		{
			return SetParameter.NAME;
		}
		else
		{
			throw new IllegalArgumentException("\"" + setParameter + "\" didn't match any set parameter!");
		}
	}

	private CommandType findCommandType(String commandType) throws IllegalArgumentException
	{
		if (commandType.equals(CommandType.CREATE.toString()))
		{
			return CommandType.CREATE;
		}
		else if (commandType.equals(CommandType.RUN.toString()))
		{
			return CommandType.RUN;
		}
		else if (commandType.equals(CommandType.STOP.toString()))
		{
			return CommandType.STOP;
		}
		else if (commandType.equals(CommandType.KILL.toString()))
		{
			return CommandType.KILL;
		}
		else if (commandType.equals(CommandType.RESTART.toString()))
		{
			return CommandType.RESTART;
		}
		else if (commandType.equals(CommandType.SET.toString()))
		{
			return CommandType.SET;
		}
		else if (commandType.equals(CommandType.PING.toString()))
		{
			return CommandType.PING;
		}
		else
		{
			throw new IllegalArgumentException("\"" + commandType + "\" didn't match any command type!");
		}
	}

	protected String[] splitCommand(String command)
	{
		return command.split("\\s+");
	}

	public CommandParser()
	{
	}

	public Command parseCommand(String command) throws IllegalArgumentException
	{
		String[] split = this.splitCommand(command);
		String commandTypeString = split[0];

		CommandType commandType = this.findCommandType(commandTypeString);

		switch (commandType)
		{
			case CREATE:
			{
				Range simulationIds = Range.parseRange(split[1]);
				return new VehicleCommand(CommandType.CREATE, simulationIds);
			}
			case RUN:
			{
				Range simulationIds = Range.parseRange(split[1]);
				return new VehicleCommand(CommandType.RUN, simulationIds);
			}
			case STOP:
			{
				Range simulationIds = Range.parseRange(split[1]);
				return new VehicleCommand(CommandType.STOP, simulationIds);
			}
			case KILL:
			{
				Range simulationIds = Range.parseRange(split[1]);
				return new VehicleCommand(CommandType.KILL, simulationIds);
			}
			case RESTART:
			{
				Range simulationIds = Range.parseRange(split[1]);
				return new VehicleCommand(CommandType.RESTART, simulationIds);
			}
			case SET:
			{
				Range simulationIds = Range.parseRange(split[1]);
				SetParameter setParameter = this.findSetParameter(split[2]);

				StringBuilder valueBuilder = new StringBuilder();

				for (int i = 3; i < split.length; ++i)
				{
					valueBuilder.append(split[i]);

					if ((i + 1) != split.length)
					{
						valueBuilder.append(' ');
					}
				}

				return new SetCommand(simulationIds, setParameter, valueBuilder.toString());
			}
			case PING:
			{
				return new PingCommand();
			}
			default:
			{
				throw new IllegalArgumentException("\"" + command + "\" didn't match any command!");
			}
		}
	}
}

package be.uantwerpen.fti.ds.sc.common;

public class CommandParser
{
	private static final String CREATE_COMMAND = "create";
	private static final String RUN_COMMAND = "run";
	private static final String STOP_COMMAND = "stop";
	private static final String KILL_COMMAND = "kill";
	private static final String RESTART_COMMAND = "restart";
	private static final String SET_COMMAND = "set";
	private static final String PING_COMMAND = "ping";

	public CommandParser()
	{
	}

	public Command parseCommand(String command)
	{
		if (command.equals(CREATE_COMMAND))
		{
			return Command.CREATE;
		}
		else if (command.equals(RUN_COMMAND))
		{
			return Command.RUN;
		}
		else if (command.equals(STOP_COMMAND))
		{
			return Command.STOP;
		}
		else if (command.equals(KILL_COMMAND))
		{
			return Command.KILL;
		}
		else if (command.equals(RESTART_COMMAND))
		{
			return Command.RESTART;
		}
		else if (command.equals(SET_COMMAND))
		{
			return Command.SET;
		}
		else if (command.equals(PING_COMMAND))
		{
			return Command.PING;
		}
		else
		{
			throw new IllegalArgumentException("\"" + command + "\" didn't match any command!");
		}
	}
}

package be.uantwerpen.fti.ds.sc.simdeployerinterface;


import be.uantwerpen.fti.ds.sc.common.Command;
import be.uantwerpen.fti.ds.sc.common.CommandParser;

public class InteractiveCommandParser extends CommandParser
{
	private static final String QUIT_COMMAND = "quit";
	private static final String HELP_COMMAND = "help";

	private InteractiveCommand translate (Command command)
	{
		switch (command)
		{
			case CREATE:
				return InteractiveCommand.CREATE;

			case SET:
				return InteractiveCommand.SET;

			case RUN:
				return InteractiveCommand.RUN;

			case STOP:
				return InteractiveCommand.STOP;

			case KILL:
				return InteractiveCommand.KILL;

			case RESTART:
				return InteractiveCommand.RESTART;

			case PING:
				return InteractiveCommand.PING;

			default:
				return InteractiveCommand.HELP;
		}
	}

	public InteractiveCommandParser()
	{
		super();
	}

	public InteractiveCommand parseInteractiveCommand(String commandString)
	{
		try
		{
			return this.translate(super.parseCommand(commandString));
		}
		// If the regular command parser fails, the command can still be an interactive command
		catch (IllegalArgumentException iae1)
		{
			if (commandString.equalsIgnoreCase(QUIT_COMMAND))
			{
				return InteractiveCommand.QUIT;
			}
			else if (commandString.equalsIgnoreCase(HELP_COMMAND))
			{
				return InteractiveCommand.HELP;
			}
			else
			{
				throw new IllegalArgumentException("\"" + commandString + "\" didn't match any interactive command!");
			}
		}
	}
}

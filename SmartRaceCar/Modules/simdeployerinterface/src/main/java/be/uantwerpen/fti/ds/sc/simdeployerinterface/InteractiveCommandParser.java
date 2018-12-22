package be.uantwerpen.fti.ds.sc.simdeployerinterface;

public class InteractiveCommandParser
{
	private static final String QUIT_COMMAND = "quit";
	private static final String HELP_COMMAND = "help";

	private CommandParser commandParser;

	private InteractiveCommand parseCommand(String interactiveCommand)
	{
		if (interactiveCommand.equalsIgnoreCase(QUIT_COMMAND))
		{
			return InteractiveCommand.QUIT;
		}
		else if (interactiveCommand.equalsIgnoreCase(HELP_COMMAND))
		{
			return InteractiveCommand.HELP;
		}
		else
		{
			throw new IllegalArgumentException("\"" + interactiveCommand + "\" didn't match any interactive command!");
		}
	}

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
		this.commandParser = new CommandParser();
	}

	public InteractiveCommand parse(String commandString)
	{
		try
		{
			return this.translate(this.commandParser.parse(commandString));
		}
		// If the regular command parser fails, the command can still be an interactive command
		catch (IllegalArgumentException iae1)
		{
			return this.parseCommand(commandString);
		}
	}
}

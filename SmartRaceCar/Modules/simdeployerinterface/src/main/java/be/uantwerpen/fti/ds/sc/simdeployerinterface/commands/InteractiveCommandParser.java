package be.uantwerpen.fti.ds.sc.simdeployerinterface.commands;


import be.uantwerpen.fti.ds.sc.common.commands.Command;
import be.uantwerpen.fti.ds.sc.common.commands.CommandParser;

public class InteractiveCommandParser extends CommandParser
{
	private InteractiveCommandType findInteractiveCommandType (String commandType) throws IllegalArgumentException
	{
		if (commandType.equalsIgnoreCase(InteractiveCommandType.HELP.toString()))
		{
			return InteractiveCommandType.HELP;
		}
		else if (commandType.equalsIgnoreCase(InteractiveCommandType.QUIT.toString()))
		{
			return InteractiveCommandType.QUIT;
		}
		else
		{
			throw new IllegalArgumentException("Command Type \"" + commandType + "\" didn't match any known command type.");
		}
	}

	public InteractiveCommandParser()
	{
		super();
	}

	public Command parseInteractiveCommand(String commandString) throws IllegalArgumentException
	{
		try
		{
			return super.parseCommand(commandString);
		}
		catch (IllegalArgumentException iae)
		{
			String[] split = super.splitCommand(commandString);
			InteractiveCommandType type = this.findInteractiveCommandType(split[0]);
			return new InteractiveCommand(type);
		}
	}
}

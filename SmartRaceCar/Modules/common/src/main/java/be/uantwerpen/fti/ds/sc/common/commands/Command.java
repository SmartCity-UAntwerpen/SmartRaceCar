package be.uantwerpen.fti.ds.sc.common.commands;

public class Command
{
	private CommandType commandType;

	protected Command(CommandType type)
	{
		this.commandType = type;
	}

	public CommandType getCommandType()
	{
		return this.commandType;
	}

	@Override
	public String toString()
	{
		return this.commandType.toString();
	}
}

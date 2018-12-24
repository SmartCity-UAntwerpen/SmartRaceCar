package be.uantwerpen.fti.ds.sc.common.commands;

public class Command
{
	public static final String ACK = "ACK";
	public static final String NACK = "NACK";

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

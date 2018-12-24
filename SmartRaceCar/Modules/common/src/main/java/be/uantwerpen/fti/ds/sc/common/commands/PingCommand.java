package be.uantwerpen.fti.ds.sc.common.commands;

public class PingCommand extends Command
{
	public static final String PONG = "pong";

	public PingCommand()
	{
		super(CommandType.PING);
	}
}

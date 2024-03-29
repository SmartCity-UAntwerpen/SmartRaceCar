package be.uantwerpen.fti.ds.sc.simdeployerinterface.commands;

import be.uantwerpen.fti.ds.sc.common.commands.Command;
import be.uantwerpen.fti.ds.sc.common.commands.CommandType;

public class SimpleInteractiveCommand extends Command
{
	private InteractiveCommandType type;

	public SimpleInteractiveCommand(InteractiveCommandType type)
	{
		super(CommandType.OTHER);
		this.type = type;
	}

	public InteractiveCommandType getInteractiveCommandType()
	{
		return this.type;
	}

	@Override
	public String toString()
	{
		return this.type.toString();
	}
}

package be.uantwerpen.fti.ds.sc.simdeployerinterface.commands;

public class InteractiveCommand extends SimpleInteractiveCommand
{
	private String argument;

	public InteractiveCommand(InteractiveCommandType type, String argument)
	{
		super(type);
		this.argument = argument;
	}

	public String getArgument()
	{
		return this.argument;
	}

	@Override
	public String toString()
	{
		return super.toString() + ' ' + this.argument;
	}
}

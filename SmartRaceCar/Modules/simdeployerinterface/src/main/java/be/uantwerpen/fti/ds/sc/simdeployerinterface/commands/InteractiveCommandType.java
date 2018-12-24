package be.uantwerpen.fti.ds.sc.simdeployerinterface.commands;

public enum InteractiveCommandType
{
	HELP,
	QUIT,
	WAIT,
	ECHO;

	@Override
	public String toString()
	{
		return this.name().toLowerCase();
	}
}

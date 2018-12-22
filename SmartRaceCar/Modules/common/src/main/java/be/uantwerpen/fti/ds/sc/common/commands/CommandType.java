package be.uantwerpen.fti.ds.sc.common.commands;

public enum CommandType
{
	CREATE,
	RUN,
	STOP,
	KILL,
	RESTART,
	SET,
	PING,
	OTHER;

	@Override
	public String toString()
	{
		return this.name().toLowerCase();
	}
}

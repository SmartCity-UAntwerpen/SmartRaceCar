package be.uantwerpen.fti.ds.sc.simdeployer.docker;

public enum CommandType
{
	REMOVE,
	RUN,
	STOP;

	@Override
	public String toString()
	{
		switch (this)
		{
			// The remove command is different because it is called "rm" in docker.
			case REMOVE:
				return "rm";

			default:
				return this.name().toLowerCase();
		}
	}
}

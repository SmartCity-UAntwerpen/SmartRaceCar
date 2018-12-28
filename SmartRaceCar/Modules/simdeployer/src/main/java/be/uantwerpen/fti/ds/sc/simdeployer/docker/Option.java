package be.uantwerpen.fti.ds.sc.simdeployer.docker;

public enum Option
{
	NAME,
	MOUNT;

	@Override
	public String toString()
	{
		return "--" + this.name().toLowerCase();
	}
}

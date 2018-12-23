package be.uantwerpen.fti.ds.sc.common.commands;

public enum SetParameter
{
	STARTPOINT,
	SPEED,
	NAME;

	@Override
	public String toString()
	{
		return this.name().toLowerCase();
	}
}

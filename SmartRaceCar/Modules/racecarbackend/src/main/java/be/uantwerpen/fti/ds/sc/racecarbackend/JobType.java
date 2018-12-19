package be.uantwerpen.fti.ds.sc.racecarbackend;

public enum JobType
{
	LOCAL,
	GLOBAL;

	@Override
	public String toString()
	{
		return this.name();
	}
}

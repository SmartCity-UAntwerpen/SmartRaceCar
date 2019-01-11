package be.uantwerpen.fti.ds.sc.racecarbackend.jobs;

public enum JobType
{
	LOCAL,
	GLOBAL;

	@Override
	public String toString()
	{
		return this.name().toLowerCase();
	}
}

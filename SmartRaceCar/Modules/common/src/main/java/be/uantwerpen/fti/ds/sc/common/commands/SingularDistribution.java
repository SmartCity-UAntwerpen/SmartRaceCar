package be.uantwerpen.fti.ds.sc.common.commands;

public class SingularDistribution implements Distribution
{
	private long value;

	public SingularDistribution (long value)
	{
		this.value = value;
	}

	public static SingularDistribution parseDistribution(String distString) throws Exception
	{
		try
		{
			long value = Long.parseLong(distString);
			return new SingularDistribution(value);
		}
		catch (NumberFormatException nfe)
		{
			throw new Exception(nfe);
		}
	}

	public static boolean isSingular(String distString)
	{
		try
		{
			Long.parseLong(distString);
			return true;
		}
		catch (NumberFormatException nfe)
		{
			return false;
		}
	}

	@Override
	public long sample()
	{
		return this.value;
	}
}

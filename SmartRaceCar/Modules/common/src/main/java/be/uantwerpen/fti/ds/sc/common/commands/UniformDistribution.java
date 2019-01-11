package be.uantwerpen.fti.ds.sc.common.commands;

import java.util.concurrent.ThreadLocalRandom;

public class UniformDistribution implements Distribution
{
	private static final String DISTRIBUTION_NAME = "uniform";

	private long low;
	private long high;

	public UniformDistribution (long low, long high)
	{
		this.low = low;
		this.high = high;
	}

	public static UniformDistribution parseDistribution(String distString) throws Exception
	{
		if (distString.startsWith(UniformDistribution.DISTRIBUTION_NAME))
		{
			String commaSeparatedValues = distString.substring(UniformDistribution.DISTRIBUTION_NAME.length(), distString.length() - 2);
			String[] nums = commaSeparatedValues.split(",");

			try
			{
				long low = Long.parseLong(nums[0]);
				long high = Long.parseLong(nums[1]);

				return new UniformDistribution(low, high);
			}
			catch (NumberFormatException nfe)
			{
				throw new Exception("Failed to parse high or low for " + UniformDistribution.DISTRIBUTION_NAME + " distribution.");
			}
			catch (ArrayIndexOutOfBoundsException aioob)
			{
				throw new Exception("" + UniformDistribution.DISTRIBUTION_NAME + " distribution is missing either high or low number.");
			}
		}
		else
		{
			throw new Exception("Failed to parse " + UniformDistribution.DISTRIBUTION_NAME + " distribution.");
		}
	}

	public static boolean isUniform(String distString)
	{
		return distString.startsWith(UniformDistribution.DISTRIBUTION_NAME);
	}

	@Override
	public long sample()
	{
		return ThreadLocalRandom.current().nextLong(this.low, this.high + 1);
	}
}

package be.uantwerpen.fti.ds.sc.common.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public interface Distribution
{
	public long sample();

	public static Distribution parse(String distString)
	{
		Logger log = LoggerFactory.getLogger(Distribution.class);

		try
		{
			if (UniformDistribution.isUniform(distString))
			{
				return UniformDistribution.parseDistribution(distString);
			}
			else if (SingularDistribution.isSingular(distString))
			{
				return SingularDistribution.parseDistribution(distString);
			}
		}
		catch (Exception e)
		{
			log.error("Failed to parse distribution.", e);
		}

		return new SingularDistribution(0);
	}
}

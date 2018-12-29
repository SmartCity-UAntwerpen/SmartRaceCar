package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.List;

public abstract class DockerOption
{
	protected OptionType optionType;


	protected DockerOption (OptionType optionType)
	{
		this.optionType = optionType;
	}

	public OptionType getOptionType()
	{
		return this.optionType;
	}

	public abstract List<String> toStringList();
}

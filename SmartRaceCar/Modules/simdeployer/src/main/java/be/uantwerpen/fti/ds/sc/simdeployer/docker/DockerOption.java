package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.List;

public abstract class DockerOption
{
	protected Option option;


	protected DockerOption (Option option)
	{
		this.option = option;
	}

	public abstract List<String> toStringList();
}

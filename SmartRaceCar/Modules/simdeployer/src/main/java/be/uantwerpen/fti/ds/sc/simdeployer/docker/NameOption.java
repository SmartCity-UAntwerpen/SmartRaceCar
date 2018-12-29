package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.ArrayList;
import java.util.List;

public class NameOption extends DockerOption
{
	private String containerName;

	public NameOption(String containerName)
	{
		super(OptionType.NAME);
		this.containerName = containerName;
	}

	public String getContainerName()
	{
		return this.containerName;
	}

	@Override
	public List<String> toStringList()
	{
		List<String> list = new ArrayList<>();
		list.add(this.optionType.toString());
		list.add(this.containerName);
		return list;
	}
}

package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.ArrayList;
import java.util.List;

public class NameOption extends DockerOption
{
	private String containerName;

	public NameOption(String containerName)
	{
		super(Option.NAME);
	}

	@Override
	public List<String> toStringList()
	{
		List<String> list = new ArrayList<>();
		list.add(this.option.toString());
		list.add(this.containerName);
		return list;
	}
}

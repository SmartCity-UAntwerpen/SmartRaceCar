package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class DockerCommandBuilder
{
	private static final String DOCKER_COMMAND = "docker";

	private CommandType commandType;
	private String imageName;
	private List<DockerOption> options;

	public DockerCommandBuilder(CommandType commandType)
	{
		this.commandType = commandType;
		this.imageName = null;
		this.options = new LinkedList<>();
	}

	public DockerCommandBuilder addOption(DockerOption option)
	{
		this.options.add(option);
		return this;
	}

	public DockerCommandBuilder setImageName(String imageName)
	{
		this.imageName = imageName;
		return this;
	}

	public List<String> toStringList()
	{
		List<String> list = new ArrayList<>();
		list.add(DOCKER_COMMAND);
		list.add(this.commandType.toString());

		for (DockerOption option: this.options)
		{
			list.addAll(option.toStringList());
		}

		switch (this.commandType)
		{
			case RUN:
				if (this.imageName == null)
				{
					throw new IllegalArgumentException("Image name wasn't set.");
				}
				else
				{
					list.add(this.imageName);
				}
				break;

			default:
				break;
		}

		// Be sure to clear the list of options and image name when we're done
		this.options.clear();
		this.imageName = null;

		return list;
	}
}

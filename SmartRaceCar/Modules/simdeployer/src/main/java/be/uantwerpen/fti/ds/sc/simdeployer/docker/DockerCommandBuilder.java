package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.util.*;

public class DockerCommandBuilder
{
	private static final String DOCKER_COMMAND = "docker";

	private CommandType commandType;
	private String imageName;
	private Map<OptionType, DockerOption> options;

	public DockerCommandBuilder(CommandType commandType)
	{
		this.commandType = commandType;
		this.imageName = null;
		this.options = new HashMap<>();
	}

	public DockerCommandBuilder addOption(DockerOption option)
	{
		this.options.put(option.getOptionType(), option);
		return this;
	}

	public DockerCommandBuilder setImageName(String imageName)
	{
		this.imageName = imageName;
		return this;
	}

	/**
	 * Converts the current settings to a list of strings.
	 * Resets the builder.
	 * @return
	 */
	public List<String> toStringList()
	{
		List<String> list = new ArrayList<>();
		list.add(DOCKER_COMMAND);
		list.add(this.commandType.toString());

		switch (this.commandType)
		{
			case RUN:
				if (this.imageName == null)
				{
					throw new IllegalArgumentException("Image name wasn't set. This is required for a run command.");
				}
				else
				{
					for (OptionType optionType: this.options.keySet())
					{
						list.addAll(this.options.get(optionType).toStringList());
					}

					list.add(this.imageName);
				}
				break;

			case STOP:
				if (!this.options.containsKey(OptionType.NAME))
				{
					throw new IllegalArgumentException("Container name option wasn't given. This is required for a stop command.");
				}
				else
				{
					NameOption nameOption = (NameOption) this.options.get(OptionType.NAME);
					list.add(nameOption.getContainerName());
				}
				break;

			case REMOVE:
				if (!this.options.containsKey(OptionType.NAME))
				{
					throw new IllegalArgumentException("Container name option wasn't given. This is required for a stop command.");
				}
				else
				{
					NameOption nameOption = (NameOption) this.options.get(OptionType.NAME);
					list.add(nameOption.getContainerName());
				}
				break;

			default:
				for (OptionType optionType: this.options.keySet())
				{
					list.addAll(this.options.get(optionType).toStringList());
				}

				break;
		}

		// Be sure to clear the list of options and image name when we're done
		this.options.clear();
		this.imageName = null;

		return list;
	}
}

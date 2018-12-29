package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

//https://docs.docker.com/storage/volumes/#use-a-read-only-volume
public class MountOption extends DockerOption
{
	private static final String TYPE_STRING = "type=";
	private static final String BIND_TYPE = "bind";
	private static final String SOURCE_STRING = "source=";
	private static final String DESTINATION_STRING = "destination=";
	private static final String READONLY_STRING = "readonly";

	private boolean readonly;
	private File hostVolume;
	private String containerVolume;

	private String generateMountString()
	{
		StringBuilder optionBuilder = new StringBuilder();
		optionBuilder.append(TYPE_STRING);
		optionBuilder.append(BIND_TYPE);
		optionBuilder.append(',');
		optionBuilder.append(SOURCE_STRING);
		optionBuilder.append(this.hostVolume.getAbsolutePath());
		optionBuilder.append(',');
		optionBuilder.append(DESTINATION_STRING);
		optionBuilder.append(this.containerVolume);

		if (this.readonly)
		{
			optionBuilder.append(',');
			optionBuilder.append(READONLY_STRING);
		}

		return optionBuilder.toString();
	}

	public MountOption(boolean readonly, String hostVolume, String containerVolume)
	{
		super(OptionType.MOUNT);
		this.readonly = readonly;
		this.hostVolume = new File(hostVolume);
		this.containerVolume = containerVolume;
	}

	@Override
	public List<String> toStringList()
	{
		List<String> list = new ArrayList<>();
		list.add(this.optionType.toString());
		list.add(this.generateMountString());
		return list;
	}
}

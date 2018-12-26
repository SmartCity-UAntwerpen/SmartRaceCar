package be.uantwerpen.fti.ds.sc.common.commands;

public class SetCommand extends VehicleCommand
{
	private SetParameter key;
	private String value;

	public SetCommand(Range simulationIds, SetParameter setParameter, String value)
	{
		super(CommandType.SET, simulationIds);
		this.key = setParameter;
		this.value = value;
	}

	public SetParameter getKey()
	{
		return this.key;
	}

	public String getValue()
	{
		return this.value;
	}

	@Override
	public String toString()
	{
		StringBuilder commandBuilder = new StringBuilder();
		commandBuilder.append(super.getCommandType().toString());
		commandBuilder.append(' ');
		commandBuilder.append(this.getSimulationId());
		commandBuilder.append(' ');
		commandBuilder.append(this.key);
		commandBuilder.append(' ');
		commandBuilder.append(this.value);
		return commandBuilder.toString();
	}
}

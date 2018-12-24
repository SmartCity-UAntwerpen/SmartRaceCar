package be.uantwerpen.fti.ds.sc.common.commands;


public class VehicleCommand extends Command
{
	private Range simulationIds;

	public VehicleCommand(CommandType commandType, Range simulationIds)
	{
		super(commandType);
		this.simulationIds = simulationIds;
	}

	public Range getSimulationId()
	{
		return this.simulationIds;
	}

	@Override
	public String toString()
	{
		return super.toString() + " " + this.simulationIds.toString();
	}
}
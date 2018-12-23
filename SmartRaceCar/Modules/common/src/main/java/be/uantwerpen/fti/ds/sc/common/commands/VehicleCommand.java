package be.uantwerpen.fti.ds.sc.common.commands;

public class VehicleCommand extends Command
{
	private long simulationId;

	public VehicleCommand(CommandType commandType, long simulationId)
	{
		super(commandType);
		this.simulationId = simulationId;
	}

	public long getSimulationId()
	{
		return this.simulationId;
	}

	@Override
	public String toString()
	{
		return super.toString() + " " + this.simulationId;
	}
}
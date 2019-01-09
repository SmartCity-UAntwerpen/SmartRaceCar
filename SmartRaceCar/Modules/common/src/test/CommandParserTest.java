import be.uantwerpen.fti.ds.sc.common.commands.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CommandParserTest
{
	private CommandParser parser;

	@Before
	public void setup()
	{
		this.parser = new CommandParser();
	}

	@Test
	public void createValidSingle()
	{
		final Range EXPECTED_SIM_ID = new Range(0);

		final String VALID_CREATE_COMMAND = "create 0";

		Command command = this.parser.parseCommand(VALID_CREATE_COMMAND);

		assertEquals(command.getCommandType(), CommandType.CREATE);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(vehicleCommand.toString(), VALID_CREATE_COMMAND);
		assertEquals(command.toString(), VALID_CREATE_COMMAND);
	}

	@Test
	public void createValidMultiple()
	{
		final Range EXPECTED_SIM_ID = new Range(0,1);

		final String VALID_CREATE_COMMAND = "create 0...1";

		Command command = this.parser.parseCommand(VALID_CREATE_COMMAND);

		assertEquals(command.getCommandType(), CommandType.CREATE);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(vehicleCommand.toString(), VALID_CREATE_COMMAND);
		assertEquals(command.toString(), VALID_CREATE_COMMAND);
	}

	@Test
	public void setValidSingleStartpoint()
	{
		final Range EXPECTED_SIM_ID = new Range(0);

		final String VALID_SET_COMMAND = "set 0 startpoint 2";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.STARTPOINT);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidSingleSpeed()
	{
		final Range EXPECTED_SIM_ID = new Range(0);

		final String VALID_SET_COMMAND = "set 0 speed 2";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.SPEED);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidSingleName()
	{
		final Range EXPECTED_SIM_ID = new Range(0);

		final String VALID_SET_COMMAND = "set 0 name simulation-1";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.NAME);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidMultipleStartpoint()
	{
		final Range EXPECTED_SIM_ID = new Range(0, 2);

		final String VALID_SET_COMMAND = "set 0...2 startpoint 2";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.STARTPOINT);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidMultipleSpeed()
	{
		final Range EXPECTED_SIM_ID = new Range(0,2);

		final String VALID_SET_COMMAND = "set 0...2 speed 2";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.SPEED);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidMultipleName()
	{
		final Range EXPECTED_SIM_ID = new Range(0, 2);

		final String VALID_SET_COMMAND = "set 0...2 name simulation-1";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getSimulationId(), EXPECTED_SIM_ID);
		assertEquals(setCommand.getKey(), SetParameter.NAME);
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void ping()
	{
		final String VALID_PING_COMMAND = "ping";

		Command command = this.parser.parseCommand(VALID_PING_COMMAND);

		assertEquals(command.getCommandType(), CommandType.PING);

		PingCommand pingCommand = (PingCommand) command;

		assertEquals(pingCommand.toString(), VALID_PING_COMMAND);
		assertEquals(command.toString(), VALID_PING_COMMAND);
	}

	@Test(expected = IllegalArgumentException.class)
	public void createInvalidCommand() throws IllegalArgumentException
	{
		final String CREATE_COMMAND_INVALID_COMMAND = "crwate 1";

		Command command = this.parser.parseCommand(CREATE_COMMAND_INVALID_COMMAND);
	}

	@Test(expected = NumberFormatException.class)
	public void rangeInvalidSimId() throws NumberFormatException
	{
		final String CREATE_COMMAND_INVALID_SIM_ID = "create f";

		Command command = this.parser.parseCommand(CREATE_COMMAND_INVALID_SIM_ID);
	}

	@Test(expected = NumberFormatException.class)
	public void rangeMissingDot() throws NumberFormatException
	{
		final String CREATE_COMMAND_MISSING_DOT = "create 0..1";

		Command command = this.parser.parseCommand(CREATE_COMMAND_MISSING_DOT);
	}

	@After
	public void cleanup()
	{
	}
}

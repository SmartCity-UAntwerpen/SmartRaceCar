import be.uantwerpen.fti.ds.sc.common.commands.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CommandParserTest
{
	private CommandParser parser;

	@Before
	public void setup() throws Exception
	{
		this.parser = new CommandParser();
	}

	@Test
	public void createValid() throws Exception
	{
		final Range EXPECTED_SIM_ID_SINGULAR = new Range(0);
		final Range EXPECTED_SIM_ID_PLURAL = new Range(0,1);

		final String VALID_CREATE_SINGULAR_COMMAND = "create 0";
		final String VALID_CREATE_PLURAL_COMMAND = "create 0...1";

		Command command = parser.parseCommand(VALID_CREATE_SINGULAR_COMMAND);

		assertEquals(command.getCommandType(), CommandType.CREATE);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.getSimulationId(), EXPECTED_SIM_ID_SINGULAR);
		assertEquals(vehicleCommand.toString(), VALID_CREATE_SINGULAR_COMMAND);
		assertEquals(command.toString(), VALID_CREATE_SINGULAR_COMMAND);

		command = parser.parseCommand(VALID_CREATE_PLURAL_COMMAND);

		assertEquals(command.getCommandType(), CommandType.CREATE);

		vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.getSimulationId(), EXPECTED_SIM_ID_PLURAL);
		assertEquals(vehicleCommand.toString(), VALID_CREATE_PLURAL_COMMAND);
		assertEquals(command.toString(), VALID_CREATE_PLURAL_COMMAND);
	}

	@Test
	public void createInvalid() throws Exception
	{
		final String CREATE_COMMAND_INVALID_SIM_ID = "create f";
		final String CREATE_COMMAND_INVALID_COMMAND = "";
		final String CREATE_COMMAND_MISSING_DOT = "create 0..1";
	}

	@After
	public void cleanup() throws Exception
	{
	}
}

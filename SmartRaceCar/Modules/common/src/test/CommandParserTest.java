import be.uantwerpen.fti.ds.sc.common.commands.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.*;

public class CommandParserTest
{
	private CommandParser parser;

	@Before
	public void setup()
	{
		this.parser = new CommandParser();
	}

	@Test
	public void createValid()
	{
		final String VALID_CREATE_COMMAND = "create 0";

		Command command = this.parser.parseCommand(VALID_CREATE_COMMAND);

		assertEquals(command.getCommandType(), CommandType.CREATE);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.toString(), VALID_CREATE_COMMAND);
		assertEquals(command.toString(), VALID_CREATE_COMMAND);
	}

	@Test
	public void setValidStartpoint()
	{
		final String VALID_SET_COMMAND = "set 0 startpoint 2";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getKey(), SetParameter.STARTPOINT);
		assertEquals(setCommand.getValue(), "2");
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidSpeed()
	{
		final String VALID_SET_COMMAND = "set 0 speed 3";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getKey(), SetParameter.SPEED);
		assertEquals(setCommand.getValue(), "3");
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test
	public void setValidName()
	{
		final String VALID_SET_COMMAND = "set 0 name simulation-1";

		Command command = this.parser.parseCommand(VALID_SET_COMMAND);

		assertEquals(command.getCommandType(), CommandType.SET);

		SetCommand setCommand = (SetCommand) command;

		assertEquals(setCommand.getKey(), SetParameter.NAME);
		assertEquals(setCommand.getValue(), "simulation-1");
		assertEquals(setCommand.toString(), VALID_SET_COMMAND);
		assertEquals(command.toString(), VALID_SET_COMMAND);
	}

	@Test (expected = IllegalArgumentException.class)
	public void setInvalidParameter()
	{
		final String INVALID_SET_COMMAND = "set 0 cookies 10";

		Command command = this.parser.parseCommand(INVALID_SET_COMMAND);
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

	@Test
	public void run()
	{
		final String VALID_RUN_COMMAND = "run 0";

		Command command = this.parser.parseCommand(VALID_RUN_COMMAND);

		assertEquals(command.getCommandType(), CommandType.RUN);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.toString(), VALID_RUN_COMMAND);
		assertEquals(command.toString(), VALID_RUN_COMMAND);
	}

	@Test
	public void stop()
	{
		final String VALID_STOP_COMMAND = "stop 0";

		Command command = this.parser.parseCommand(VALID_STOP_COMMAND);

		assertEquals(command.getCommandType(), CommandType.STOP);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.toString(), VALID_STOP_COMMAND);
		assertEquals(command.toString(), VALID_STOP_COMMAND);
	}

	@Test
	public void kill()
	{
		final String VALID_KILL_COMMAND = "kill 0";

		Command command = this.parser.parseCommand(VALID_KILL_COMMAND);

		assertEquals(command.getCommandType(), CommandType.KILL);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.toString(), VALID_KILL_COMMAND);
		assertEquals(command.toString(), VALID_KILL_COMMAND);
	}

	@Test
	public void restart()
	{
		final String VALID_RESTART_COMMAND = "restart 0";

		Command command = this.parser.parseCommand(VALID_RESTART_COMMAND);

		assertEquals(command.getCommandType(), CommandType.RESTART);

		VehicleCommand vehicleCommand = (VehicleCommand) command;

		assertEquals(vehicleCommand.toString(), VALID_RESTART_COMMAND);
		assertEquals(command.toString(), VALID_RESTART_COMMAND);
	}

	@Test(expected = IllegalArgumentException.class)
	public void invalidCommand() throws IllegalArgumentException
	{
		final String INVALID_COMMAND = "crwate 1";

		Command command = this.parser.parseCommand(INVALID_COMMAND);
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

	@Test
	public void rangeEqualsObject()
	{
		final Object OBJECT = new Object();
		final Range RANGE = new Range(5);

		assertFalse(RANGE.equals(OBJECT));
	}

	@Test
	public void rangeEqualsRange()
	{
		final Range RANGE1 = new Range(1, 10);
		final Range RANGE2 = new Range(1, 10);

		assertTrue(RANGE1.equals(RANGE2));
	}

	@Test
	public void rangeToString()
	{
		final String RANGE_STRING = "0...3";

		Range range = new Range(0,3);

		assertEquals(RANGE_STRING, range.toString());
	}

	@Test (expected = NumberFormatException.class)
	public void invalidMultiRange()
	{
		final String INVALID_MULTI_RANGE = "0...3...5";

		Range range = Range.parseRange(INVALID_MULTI_RANGE);
	}

	@Test
	public void generateRange()
	{
		final String RANGE_STRING = "0...2";

		Set<Long> EXPECTED_NUMBERS = new HashSet<Long>();
		EXPECTED_NUMBERS.add(0L);
		EXPECTED_NUMBERS.add(1L);
		EXPECTED_NUMBERS.add(2L);

		Range range = Range.parseRange(RANGE_STRING);

		assertTrue(range.generate().equals(EXPECTED_NUMBERS));
	}

	@After
	public void cleanup()
	{
	}
}

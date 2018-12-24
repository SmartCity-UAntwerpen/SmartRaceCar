package be.uantwerpen.fti.ds.sc.simdeployerinterface.commands;


import be.uantwerpen.fti.ds.sc.common.commands.Command;
import be.uantwerpen.fti.ds.sc.common.commands.CommandParser;

import java.text.ParseException;

public class InteractiveCommandParser extends CommandParser
{
	private String parseEchoArgument(String[] parts) throws ParseException
	{
		// Reassemble all parts, separate by spaces
		StringBuilder stringBuilder = new StringBuilder();

		for (int i = 0; i < parts.length; ++i)
		{
			stringBuilder.append(parts[i]);

			// Append spaces after every part but the last
			if ((i + 1) != parts.length)
			{
				stringBuilder.append(' ');
			}
		}

		// Check if quotes are correctly matched
		boolean openQuote = false;              // Check if all quotes are matched
		String joinedString = stringBuilder.toString();

		for (int i = 0; i < joinedString.length(); ++i)
		{
			if (i > 0)
			{
				if ((joinedString.charAt(i) == '"') && (joinedString.charAt(i - 1) != '\\'))
				{
					openQuote = !openQuote;
				}
			}
			else
			{
				// First character, Only append if its not a "
				if (joinedString.charAt(i) == '"')
				{
					openQuote = !openQuote;
				}
			}
		}

		if (openQuote)
		{
			throw new ParseException("Failed to parse argument for echo command, argument: '" + joinedString + "'. Found mismatched quotes.", 0);
		}

		// Removed escape characters from strings
		StringBuilder finalStingBuilder = new StringBuilder();

		// Quotes are matched, remove non-escaped quotes.
		for (int i = 0; i < joinedString.length(); ++i)
		{
			if (i > 0)
			{
				if (joinedString.charAt(i) == '\\')
				{
					// We found an escape character, Ignore it
				}
				else if ((joinedString.charAt(i) == '"') && (joinedString.charAt(i - 1) != '\\'))
				{
					// We found an un-escaped ", Ignore it
				}
				else
				{
					finalStingBuilder.append(joinedString.charAt(i));
				}
			}
			else
			{
				// First character, Only append if its not a "
				if (joinedString.charAt(i) != '"')
				{
					finalStingBuilder.append(joinedString);
				}
			}
		}

		return finalStingBuilder.toString();
	}

	private InteractiveCommandType findInteractiveCommandType (String commandType) throws IllegalArgumentException
	{
		if (commandType.equalsIgnoreCase(InteractiveCommandType.HELP.toString()))
		{
			return InteractiveCommandType.HELP;
		}
		else if (commandType.equalsIgnoreCase(InteractiveCommandType.QUIT.toString()))
		{
			return InteractiveCommandType.QUIT;
		}
		else if (commandType.equalsIgnoreCase((InteractiveCommandType.ECHO.toString())))
		{
			return InteractiveCommandType.ECHO;
		}
		else if (commandType.equalsIgnoreCase((InteractiveCommandType.WAIT.toString())))
		{
			return InteractiveCommandType.WAIT;
		}
		else
		{
			throw new IllegalArgumentException("Command Type \"" + commandType + "\" didn't match any known command type.");
		}
	}

	public InteractiveCommandParser()
	{
		super();
	}

	public Command parseInteractiveCommand(String commandString) throws IllegalArgumentException, ParseException
	{
		try
		{
			return super.parseCommand(commandString);
		}
		catch (IllegalArgumentException iae)
		{
			String[] split = super.splitCommand(commandString);
			InteractiveCommandType type = this.findInteractiveCommandType(split[0]);

			switch (type)
			{
				case WAIT:
					return new InteractiveCommand(InteractiveCommandType.WAIT, split[1]);
				case ECHO:
					String[] argParts = new String [split.length - 1];
					System.arraycopy(split, 1, argParts, 0, split.length - 1);
					return new InteractiveCommand(InteractiveCommandType.ECHO, this.parseEchoArgument(argParts));

				default:
					return new SimpleInteractiveCommand(type);
			}
		}
	}
}

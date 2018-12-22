package be.uantwerpen.fti.ds.sc.simdeployerinterface;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class CommandLineInterface
{
	private static final String[] SET_SUB_COMMANDS = {"startpoint", "speed", "name"};

	private static String help()
	{
		Map<String, String> helpMap = new HashMap<>();

		helpMap.put("create", "Create a simulation instance. This just announces the ID of the simulation to the SimDeployer.");
		helpMap.put("set", "Set a parameter of the simulation. Currently only \"startpoint\" is supported. \"speed\" and \"name\" will be parsed correctly, but don't have any effect.");
		helpMap.put("run", "Run a simulation instance created using the \"create\" command. Requires that the simulation has a start point. This starts an actual Docker container.");
		helpMap.put("stop", "Kill a simulation instance. The corresponding Docker process will be killed and the simulation will lose any held state.");
		helpMap.put("kill", "Remove a simulation instance from the list of IDs.");
		helpMap.put("restart", "Same as run.");
		helpMap.put("ping", "Used by Simulation front end to keep an eye on SimDeployers. Returns \"pong\".");
		helpMap.put("help", "Show this help message.");
		helpMap.put("quit", "Kill the interactive console session.");

		StringBuilder helpBuilder = new StringBuilder();
		for (String command: helpMap.keySet())
		{
			helpBuilder.append(command);
			helpBuilder.append("\t\t");
			helpBuilder.append(helpMap.get(command));
			helpBuilder.append('\n');
		}

		return helpBuilder.toString();
	}

	public static void main(String[] args)
	{
		Scanner scanner = new Scanner(System.in);
		InteractiveCommandParser commandParser = new InteractiveCommandParser();
		boolean quit = false;

		System.out.println("Welcome to the SmartCity SimDeployer Commandline utility.");
		System.out.println("Type \"help\" for a list of possible commands.");

		while (!quit)
		{
			String line = scanner.nextLine();
			String[] parts = line.split("\\s"); // Split the input on any whitespace

			InteractiveCommand command = null;

			try
			{
				command = commandParser.parse(parts[0]);
			}
			catch (IllegalArgumentException iae)
			{
				System.err.println("\"" + line + "\" is not a valid command.");
				continue;
			}

			String response = "";

			switch (command)
			{
				case CREATE:
				{
					long simulationId = Long.parseLong(parts[1]);
					break;
				}

				case SET:
				{
					break;
				}

				case RUN:
				{
					long simulationId = Long.parseLong(parts[1]);
					break;
				}

				case HELP:
				{
					response = help();
					break;
				}

				case KILL:
					break;

				case PING:
					break;

				case QUIT:
					break;

				case STOP:
					break;

				case RESTART:
					break;

				default:
					continue;
			}

			System.out.println(response);
		}
	}
}

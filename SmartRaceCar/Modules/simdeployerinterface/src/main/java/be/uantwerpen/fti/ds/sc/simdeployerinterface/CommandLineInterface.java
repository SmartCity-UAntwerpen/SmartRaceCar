package be.uantwerpen.fti.ds.sc.simdeployerinterface;

import java.io.*;
import java.util.*;

public class CommandLineInterface
{
	private static final String JAR_NAME = "SimDeployerInterface.jar";

	private static final String STARTPOINT_KEY = "startpoint";
	private static final String SPEED_KEY = "speed";
	private static final String NAME_KEY = "name";

	private static String generateSetCommand (String[] args)
	{
		String key = args[0];
		long simulationID = Long.parseLong(args[1]);
		String value = args[2];

		if (key.equalsIgnoreCase(STARTPOINT_KEY))
		{
			return "set startpoint " + simulationID + " " + Long.parseLong(value);
		}
		else if (key.equalsIgnoreCase(SPEED_KEY))
		{
			return "set speed " + simulationID + " " + Float.parseFloat(value);
		}
		else if (key.equalsIgnoreCase(NAME_KEY))
		{
			return "set name " + simulationID + " " + value;
		}

		return "";
	}

	private static String sendCommand(String command, String remoteHost, int remotePort) throws IOException
	{
		TCPClient client = new TCPClient(remoteHost, remotePort);
		client.send(command);
		String response = client.receive();
		client.close();
		return response;
	}

	private static void usage()
	{
		System.out.println("java -jar " + JAR_NAME + " [REMOTE HOST] [REMOTE TCP PORT] [SCRIPT FILE]");
		System.out.println("\tREMOTE HOST\t\tThe host the SimDeployer is running on.");
		System.out.println("\tREMOTE TCP PORT\t\tThe port the SimDeployer is running on.");
		System.out.println("\tSCRIPT FILE\t\tOptional. A script to be used instead of an interactive command line.");
	}

	private static String help()
	{
		Map<String, String> helpMap = new HashMap<>();

		// Add some extra space after each command to make them all the same length
		helpMap.put("create ", "Create a simulation instance. This just announces the ID of the simulation to the SimDeployer.");
		helpMap.put("set    ", "Set a parameter of the simulation. Currently only \"startpoint\" is supported. \"speed\" and \"name\" will be parsed correctly, but don't have any effect.");
		helpMap.put("run    ", "Run a simulation instance created using the \"create\" command. Requires that the simulation has a start point. This starts an actual Docker container.");
		helpMap.put("stop   ", "Kill a simulation instance. The corresponding Docker process will be killed and the simulation will lose any held state.");
		helpMap.put("kill   ", "Remove a simulation instance from the list of IDs.");
		helpMap.put("restart", "Same as run.");
		helpMap.put("ping   ", "Used by Simulation front end to keep an eye on SimDeployers. Returns \"pong\".");
		helpMap.put("help   ", "Show this help message.");
		helpMap.put("quit   ", "Kill the interactive console session.");

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
		if (args.length < 2)
		{
			usage();
		}

		final String remoteHost = args[0];
		final int remotePort = Integer.parseInt(args[1]);

		Scanner scanner = null;

		boolean interactiveMode = true;
		if (args.length == 3)
		{
			try
			{
				System.out.println("Attempting to read script \"" + args[2] + "\"");
				scanner = new Scanner(new File(args[2]));
			}
			catch (FileNotFoundException fnfe)
			{
				System.out.println("Failed to open script file: " + fnfe.getMessage());
				fnfe.printStackTrace();
				System.exit(-1);
			}

			// We succesfully hooked up the scanner to the file.
			interactiveMode = false;
		}
		else
		{
			scanner = new Scanner(System.in);
		}

		InteractiveCommandParser commandParser = new InteractiveCommandParser();
		boolean quit = false;

		if (interactiveMode)
		{
			System.out.println("Welcome to the SmartCity SimDeployer Commandline utility.");
			System.out.println("Type \"help\" for a list of possible commands.");
		}

		while (!quit)
		{
			String line = "";

			try
			{
				line = scanner.nextLine();
			}
			catch (NoSuchElementException nsee)
			{
				System.out.println("Read final line, exiting...");
				quit = true;
				continue;
			}

			if (!interactiveMode)
			{
				System.out.println(line);
			}

			String[] parts = line.split("\\s"); // Split the input on any whitespace

			InteractiveCommand command = null;

			try
			{
				command = commandParser.parseInteractiveCommand(parts[0]);
			}
			catch (IllegalArgumentException iae)
			{
				System.err.println("\"" + line + "\" is not a valid command.");
				continue;
			}

			String response = "";

			try
			{
				switch (command)
				{
					case CREATE:
					{
						long simulationId = Long.parseLong(parts[1]);
						response = sendCommand("create " + simulationId, remoteHost, remotePort);
						break;
					}

					case SET:
					{
						String[] commandArgs = new String [parts.length - 1];
						for (int i = 1; i < parts.length; ++i)
						{
							commandArgs[i-1] = parts[i];
						}
						response = sendCommand(generateSetCommand(commandArgs), remoteHost, remotePort);
						break;
					}

					case RUN:
					{
						long simulationId = Long.parseLong(parts[1]);
						response = sendCommand("run " + simulationId, remoteHost, remotePort);

						break;
					}

					case HELP:
					{
						response = help();
						break;
					}

					case KILL:
					{
						long simulationId = Long.parseLong(parts[1]);
						response = sendCommand("kill " + simulationId, remoteHost, remotePort);
						break;
					}

					case PING:
					{
						response = sendCommand("ping", remoteHost, remotePort);
						break;
					}

					case QUIT:
					{
						quit = true;
						continue;
					}

					case STOP:
					{
						long simulationId = Long.parseLong(parts[1]);
						response = sendCommand("stop " + simulationId, remoteHost, remotePort);
						break;
					}

					case RESTART:
					{
						long simulationId = Long.parseLong(parts[1]);
						response = sendCommand("restart " + simulationId, remoteHost, remotePort);
						break;
					}

					default:
						continue;
				}
			}
			catch (IOException ioe)
			{
				System.out.println(ioe.getMessage());
				ioe.printStackTrace();
				continue;
			}

			System.out.println(response);
		}
	}
}

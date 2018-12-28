package be.uantwerpen.fti.ds.sc.simdeployerinterface;

import be.uantwerpen.fti.ds.sc.common.commands.Command;
import be.uantwerpen.fti.ds.sc.common.commands.CommandType;
import be.uantwerpen.fti.ds.sc.simdeployerinterface.commands.InteractiveCommand;
import be.uantwerpen.fti.ds.sc.simdeployerinterface.commands.SimpleInteractiveCommand;
import be.uantwerpen.fti.ds.sc.simdeployerinterface.commands.InteractiveCommandParser;

import java.io.*;
import java.text.ParseException;
import java.util.*;

public class CommandLineInterface
{
	private static final String JAR_NAME = "SimDeployerInterface.jar";

	private String remoteHost;
	private int remotePort;
	private boolean interactiveMode;
	private File scriptFile;

	private String sendCommand(Command command, String remoteHost, int remotePort) throws IOException
	{
		TCPClient client = new TCPClient(remoteHost, remotePort);
		client.send(command.toString());
		String response = client.receive();
		client.close();
		return response;
	}

	private String help()
	{
		Map<String, String> helpMap = new TreeMap<>();  // TreeMap is used to sort commands alphabetically.

		// Add some extra space after each command to make them all the same length
		helpMap.put("create        SIM_ID         ", "Create a simulation instance. This just announces the ID of the simulation to the SimDeployer.");
		helpMap.put("set    SIM_ID PROPERTY VALUE ", "Set a parameter of the simulation. Currently only \"startpoint\" is supported. \"speed\" and \"name\" will be parsed correctly, but don't have any effect.");
		helpMap.put("run           SIM_ID         ", "Run a simulation instance created using the \"create\" command. Requires that the simulation has a start point. This starts an actual Docker container.");
		helpMap.put("stop          SIM_ID         ", "Kill a simulation instance. The corresponding Docker process will be killed and the simulation will lose any held state.");
		helpMap.put("kill          SIM_ID         ", "Remove a simulation instance from the list of IDs.");
		helpMap.put("restart       SIM_ID         ", "Same as run.");
		helpMap.put("ping                         ", "Used by Simulation front end to keep an eye on SimDeployers. Returns \"pong\".");
		helpMap.put("help                         ", "Show this help message.");
		helpMap.put("quit                         ", "Kill the interactive console session. (Leaves the SimDeployer running as normal, doesn't kill any simulations.)");
		helpMap.put("echo                 STRING  ", "Print a given string. Multi-word strings should be properly quoted, escaped quotes are reduced to normal quotes.");
		helpMap.put("wait                 TIME    ", "Wait the specified number of seconds.");

		StringBuilder helpBuilder = new StringBuilder();

		helpBuilder.append("Simulation IDs can be specified as a single number (\"1\" for example) or as an inclusive range, with high and low separated by an ellision sign. (\"0...10\" for example (This is equivalent to [0,10])).");
		helpBuilder.append("\n\n");

		for (String command: helpMap.keySet())
		{
			helpBuilder.append(command);
			helpBuilder.append("\t");
			helpBuilder.append(helpMap.get(command));
			helpBuilder.append('\n');
		}

		return helpBuilder.toString();
	}

	public void loop() throws FileNotFoundException
	{
		Scanner scanner = null;

		if (this.interactiveMode)
		{
			scanner = new Scanner(System.in);
		}
		else
		{
			scanner = new Scanner(this.scriptFile);
		}

		InteractiveCommandParser commandParser = new InteractiveCommandParser();

		if (this.interactiveMode)
		{
			System.out.println("Welcome to the SmartCity SimDeployer Commandline utility.");
			System.out.println("Type \"help\" for a list of possible commands.");
		}

		while (true)
		{
			String line = "";

			try
			{
				line = scanner.nextLine();
			}
			catch (NoSuchElementException nsee)
			{
				System.out.println("Read final line, exiting...");
				return;
			}

			Command command = null;

			try
			{
				command = commandParser.parseInteractiveCommand(line);
			}
			catch (IllegalArgumentException | ParseException iae)
			{
				System.err.println("\"" + line + "\" is not a valid command.");
				iae.printStackTrace();
				continue;
			}

			// The return value for the command is captured in this variable and displayed at the end of the loop
			String response = "";

			try
			{
				if (command.getCommandType() == CommandType.OTHER)
				{
					SimpleInteractiveCommand simpleInteractiveCommand = (SimpleInteractiveCommand) command;

					switch (simpleInteractiveCommand.getInteractiveCommandType())
					{
						case HELP:
							response = help();
							break;

						case ECHO:
						{
							InteractiveCommand interactiveCommand = (InteractiveCommand) simpleInteractiveCommand;
							response = interactiveCommand.getArgument();
							break;
						}

						case WAIT:
						{
							InteractiveCommand interactiveCommand = (InteractiveCommand) simpleInteractiveCommand;
							response = Command.ACK;
							Thread.sleep(Long.parseLong(interactiveCommand.getArgument()) * 1000L);
							break;
						}

						case QUIT:
							return;
					}
				}
				else
				{
					response = this.sendCommand(command, remoteHost, remotePort);
				}
			}
			catch (IOException | InterruptedException ie)
			{
				System.err.println(ie.getMessage());
				ie.printStackTrace();
				continue;
			}

			if (this.interactiveMode)
			{
				System.out.println(response);
			}
		}
	}

	public CommandLineInterface(String remoteHost, int remotePort)
	{
		this.remoteHost = remoteHost;
		this.remotePort = remotePort;
		this.interactiveMode = true;
		this.scriptFile = null;
	}

	public CommandLineInterface(String remoteHost, int remotePort, String scriptFilename)
	{
		this(remoteHost, remotePort);
		this.interactiveMode = false;
		this.scriptFile = new File(scriptFilename);
	}

	private static void usage()
	{
		System.out.println("java -jar " + JAR_NAME + " [REMOTE HOST] [REMOTE TCP TCP_CLIENT] [SCRIPT FILE]");
		System.out.println("\tREMOTE HOST\t\tThe host the SimDeployer is running on.");
		System.out.println("\tREMOTE TCP TCP_CLIENT\t\tThe port the SimDeployer is running on.");
		System.out.println("\tSCRIPT FILE\t\tOptional. A script to be used instead of an interactive command line.");
	}

	public static void main(String[] args)
	{
		if (args.length < 2)
		{
			usage();
			System.exit(-1);
		}

		final String remoteHost = args[0];
		final int remotePort = Integer.parseInt(args[1]);

		CommandLineInterface cli = null;

		if (args.length == 3)
		{
			String scriptFile = args[2];
			cli = new CommandLineInterface(remoteHost, remotePort, scriptFile);
		}
		else
		{
			cli = new CommandLineInterface(remoteHost, remotePort);
		}

		try
		{
			cli.loop();
		}
		catch (FileNotFoundException fnfe)
		{
			System.err.println("Failed to find Script file: " + fnfe.getMessage());
			fnfe.printStackTrace();
			System.exit(-1);
		}
	}
}

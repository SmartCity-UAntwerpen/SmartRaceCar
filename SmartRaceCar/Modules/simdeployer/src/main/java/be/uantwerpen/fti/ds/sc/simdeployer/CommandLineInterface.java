package be.uantwerpen.fti.ds.sc.simdeployer;

import org.jline.terminal.Terminal;
import org.jline.terminal.TerminalBuilder;

import java.io.IOException;

public class CommandLineInterface
{
	private static final String PROMPT = "SimDeployer > ";

	public static void main(String[] args)
	{
		final SimDeployerParameters parameters = new SimDeployerParameters(9999, "ubuntu:14.04");

		SimDeployerV2 simDeployer;

		try
		{
			simDeployer = new SimDeployerV2(parameters);
		}
		catch (IOException ioe)
		{
			System.err.println(ioe.getMessage());
			ioe.printStackTrace();
			System.out.println("Failed to start SimDeployer, exiting...");
			System.exit(-1);
		}

		boolean quit = false;

		TerminalBuilder terminalBuilder = TerminalBuilder.builder();
		Terminal commandLineTerminal;

		try
		{
			// Set up a command-line terminal
			commandLineTerminal = terminalBuilder.system(true).build();
		}
		catch (IOException ioe)
		{
			System.err.println(ioe.getMessage());
			ioe.printStackTrace();
			System.out.println("Failed to set up system terminal, exiting...");
			System.exit(-1);
		}


	}
}

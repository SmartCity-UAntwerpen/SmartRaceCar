package be.uantwerpen.fti.ds.sc.core;

import java.io.IOException;

public class Main
{
	/**
	 * Main method, used to create a Core object and run it.
	 *
	 * @param args required arguments: startpoint, tcpclientport and tcpserverport
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws IOException, InterruptedException
	{
		String propertyPath = "./";
		long startPoint = 0;

		if (args.length != 2)
		{
			System.out.println("Need 2 arguments to run. Required arguments: startpoint(int), property path (string)");
			System.exit(0);
		}
		else
		{
			if (!args[0].isEmpty()) startPoint = Long.parseLong(args[0]);
			if (!args[1].isEmpty()) propertyPath = args[1];
		}

		final Core core = new Core(startPoint, propertyPath);
	}
}

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
		int clientPort = 5005;
		int serverPort = 5006;
		long startPoint = 0;

		if (args.length == 0)
		{
			System.out.println("Need at least 1 or 3 argument to run. Possible arguments: startpoint(int)(needed!) tcpclientport(int) tcpserverport(int)");
			System.exit(0);
		} else if (args.length == 1)
		{
			if (!args[0].isEmpty()) startPoint = Long.parseLong(args[0]);
		} else if (args.length == 2)
		{
			System.out.println("Need at least 1 or 3 argument to run. Possible arguments: startpoint(int)(needed!) tcpclientport(int) tcpserverport(int)");
			System.exit(0);
		} else
		{
			if (!args[0].isEmpty()) startPoint = Long.parseLong(args[0]);
			if (!args[1].isEmpty()) serverPort = Integer.parseInt(args[1]);
			if (!args[2].isEmpty()) clientPort = Integer.parseInt(args[2]);
		}
		final Core core = new Core(startPoint, serverPort, clientPort);
	}
}

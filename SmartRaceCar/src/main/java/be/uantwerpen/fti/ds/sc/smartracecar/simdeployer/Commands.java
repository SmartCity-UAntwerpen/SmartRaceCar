package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import java.util.Scanner;

/**
 * Helper class to locally test the SimDeployer class
 */
public class Commands
{
	public static void main(String[] args) throws Exception
	{
		SimDeployer simDeployer = new SimDeployer();
		Scanner scanner = new Scanner(System.in);
		boolean running = true;
		String command;
		String[] commandArray;
		int counter = 0;
		int id;
		System.out.println("You can enter commands now.");

		while (running)
		{
			command = scanner.nextLine();
			commandArray = command.split(" ");
			switch (commandArray[0])
			{
				case "create":
					if (simDeployer.parseTCP("create " + counter).equals("ACK"))
						System.out.println("Succesfully created car with id " + counter);
					else
						System.out.println("car creation failed");
					counter++;
					break;
				case "run":
					id = Integer.parseInt(commandArray[1]);
					if (simDeployer.parseTCP("run " + id).equals("ACK"))
						System.out.println("Succesfully ran car with id " + id);
					else
						System.out.println("car run failed");
					break;
				case "stop":
					id = Integer.parseInt(commandArray[1]);
					if (simDeployer.parseTCP("stop " + id).equals("ACK"))
						System.out.println("Succesfully stopped car with id " + id);
					else
						System.out.println("car stop failed");
					break;
				case "kill":
					id = Integer.parseInt(commandArray[1]);
					if (simDeployer.parseTCP("kill " + id).equals("ACK"))
						System.out.println("Succesfully killed car with id " + id);
					else
						System.out.println("car kill failed");
					break;
				case "restart":
					id = Integer.parseInt(commandArray[1]);
					if (simDeployer.parseTCP("restart " + id).equals("ACK"))
						System.out.println("Succesfully restarted car with id " + id);
					else
						System.out.println("car restart failed");
					break;
				case "set":
					id = Integer.parseInt(commandArray[1]);
					if (simDeployer.parseTCP(command).equals("ACK"))
						System.out.println("Succesfully set car with id " + id);
					else
						System.out.println("car setup failed");
					break;
				case "exit": //kill all cars on exit
					running = false;
					for (int i = 0; i < counter; i++)
					{
						if (simDeployer.parseTCP("kill " + i).equals("ACK"))
							System.out.println("Succesfully killed car with id " + i);
						else
							System.out.println("car kill failed");
					}
					System.exit(0);
					break;
				default:
					System.out.println(command + " is not a valid command");

			}
		}
	}
}

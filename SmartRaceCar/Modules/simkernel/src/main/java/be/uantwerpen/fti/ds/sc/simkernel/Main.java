package be.uantwerpen.fti.ds.sc.simkernel;

public class Main
{
    /**
     * Main method to run this class as a separate jar file
     *
     * @param args tcpclientport and tcpserverport required
     * @throws Exception
     */
    public static void main(String[] args) throws Exception
    {
        if (args.length != 3)
        {
            System.out.println("Need 2 arguments to run. Possible arguments: tcpclientport(int) tcpserverport(int)");
            //System.exit(0);
        }
        else
        {
            int serverPort = 0;
            int clientPort = 0;
            long simID = 0;

            if (!args[0].isEmpty())
            {
                serverPort = Integer.parseInt(args[0]);
            }
            if (!args[1].isEmpty())
            {
                clientPort = Integer.parseInt(args[1]);
            }
            if(!args[2].isEmpty())
			{
				simID = Long.parseLong(args[2]);
			}

            final SimKernel simKernel = new SimKernel(serverPort, clientPort, simID);
        }
    }
}

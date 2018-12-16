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
        if (args.length != 2)
        {
            System.out.println("Need 2 arguments to run. Possible arguments: tcpclientport(int) tcpserverport(int)");
            //System.exit(0);
        } else if (args.length == 2)
        {
            int serverPort = 0;
            int clientPort = 0;

            if (!args[0].isEmpty())
            {
                serverPort = Integer.parseInt(args[0]);
            }
            if (!args[1].isEmpty())
            {
                clientPort = Integer.parseInt(args[1]);
            }

            final SimKernel simKernel = new SimKernel(serverPort, clientPort);
        }
    }
}

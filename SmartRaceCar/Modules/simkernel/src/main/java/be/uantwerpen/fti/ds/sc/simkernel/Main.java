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
            System.out.println("Need 2 arguments to run. Required arguments: simulation ID (long), property path (string)");
            //System.exit(0);
        }
        else
        {
            String propertyPath = "./";
            long simID = 0;


            if(!args[0].isEmpty())
			{
				simID = Long.parseLong(args[2]);
			}
			if(!args[1].isEmpty())
			{
				propertyPath = args[1];
			}

            final SimKernel simKernel = new SimKernel(simID, propertyPath);
        }
    }
}

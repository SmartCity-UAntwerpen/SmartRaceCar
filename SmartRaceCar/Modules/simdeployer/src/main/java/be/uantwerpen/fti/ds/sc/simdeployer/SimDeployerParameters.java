package be.uantwerpen.fti.ds.sc.simdeployer;

public class SimDeployerParameters
{
	private int serverPort;

	public SimDeployerParameters(int serverPort)
	{
		this.serverPort = serverPort;
	}

	public int getServerPort()
	{
		return this.serverPort;
	}
}

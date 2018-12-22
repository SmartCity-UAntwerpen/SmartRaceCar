package be.uantwerpen.fti.ds.sc.simdeployer;

public class SimDeployerParameters
{
	private static final int DEFAULT_PORT = 9999;
	private static final String DEFAULT_DOCKER_IMAGE = "astridvanneste/core_simkernel";

	private int serverPort;
	private String dockerImage;

	public SimDeployerParameters()
	{
		this(DEFAULT_PORT, DEFAULT_DOCKER_IMAGE);
	}

	public SimDeployerParameters(int serverPort, String dockerImage)
	{
		this.serverPort = serverPort;
		this.dockerImage = dockerImage;
	}

	public int getServerPort()
	{
		return this.serverPort;
	}

	public String getDockerImage()
	{
		return this.dockerImage;
	}
}

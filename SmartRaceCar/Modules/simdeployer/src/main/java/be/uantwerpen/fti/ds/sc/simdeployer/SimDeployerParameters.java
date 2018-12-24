package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.Parameters;

public class SimDeployerParameters extends Parameters
{
	private static final int DEFAULT_PORT = 9999;
	private static final String DEFAULT_DOCKER_IMAGE = "astridvanneste/core_simkernel";

	private int serverPort;
	private String dockerImage;

	public SimDeployerParameters()
	{
		this(new Parameters(), DEFAULT_PORT, DEFAULT_DOCKER_IMAGE);
	}

	public SimDeployerParameters(Parameters parameters, int serverPort, String dockerImage)
	{
		super(parameters);
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

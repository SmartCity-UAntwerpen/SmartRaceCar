package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class DockerAspect extends Aspect
{
	private static final String PREFIX = "Docker";

	private static final String DOCKER_IMAGE_NAME_KEY = PREFIX + ".image";

	private static final String DEFAULT_DOCKER_IMAGE_NAME = "astridvanneste/core_simkernel";

	private Logger log;
	private String imageName;

	public DockerAspect (File configFile) throws IOException
	{
		super(AspectType.DOCKER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.imageName = properties.getProperty(DOCKER_IMAGE_NAME_KEY, DEFAULT_DOCKER_IMAGE_NAME);

			this.log.debug(DOCKER_IMAGE_NAME_KEY + " = " + this.imageName);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public DockerAspect (String imageName)
	{
		super(AspectType.DOCKER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		this.imageName = imageName;
		this.log.debug(DOCKER_IMAGE_NAME_KEY + " = " + this.imageName);
	}

	public String getImageName()
	{
		return this.imageName;
	}
}

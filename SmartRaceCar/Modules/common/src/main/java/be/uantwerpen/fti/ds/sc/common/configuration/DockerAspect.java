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
	private static final String DOCKER_VOLUME_READONLY_KEY = PREFIX + ".volume.readonly";
	private static final String DOCKER_VOLUME_HOST_KEY = PREFIX + ".volume.host";
	private static final String DOCKER_VOLUME_CONTAINER_KEY = PREFIX + ".volume.container";
	private static final String[] KEYS = {DOCKER_IMAGE_NAME_KEY, DOCKER_VOLUME_READONLY_KEY, DOCKER_VOLUME_HOST_KEY, DOCKER_VOLUME_CONTAINER_KEY};

	private static final String DEFAULT_DOCKER_IMAGE_NAME = "astridvanneste/core_simkernel";
	private static final String DEFAULT_DOCKER_VOLUME_READONLY = "true";
	private static final String DEFAULT_DOCKER_VOLUME_HOST = "docker/config";
	private static final String DEFAULT_DOCKER_VOLUME_CONTAINER = "/home/docker/config";

	private Logger log;
	private boolean readonly;
	private String imageName;
	private String hostVolume;
	private String containerVolume;

	public DockerAspect (File configFile) throws IOException
	{
		super(AspectType.DOCKER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.imageName = properties.getProperty(DOCKER_IMAGE_NAME_KEY, DEFAULT_DOCKER_IMAGE_NAME);
			this.hostVolume = properties.getProperty(DOCKER_VOLUME_HOST_KEY, DEFAULT_DOCKER_VOLUME_HOST);
			this.containerVolume = properties.getProperty(DOCKER_VOLUME_CONTAINER_KEY, DEFAULT_DOCKER_VOLUME_CONTAINER);
			this.readonly = Boolean.parseBoolean(properties.getProperty(DOCKER_VOLUME_READONLY_KEY, DEFAULT_DOCKER_VOLUME_READONLY));

			this.log.debug(DOCKER_IMAGE_NAME_KEY + " = " + this.imageName);
			this.log.debug(DOCKER_VOLUME_HOST_KEY + " = " + this.hostVolume);
			this.log.debug(DOCKER_VOLUME_CONTAINER_KEY + " = " + this.containerVolume);
			this.log.debug(DOCKER_VOLUME_READONLY_KEY + " = " + this.readonly);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public DockerAspect (String imageName, String hostVolume, String containerVolume)
	{
		super(AspectType.DOCKER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		this.imageName = imageName;
		this.hostVolume = hostVolume;
		this.containerVolume = containerVolume;

		this.log.debug(DOCKER_IMAGE_NAME_KEY + " = " + this.imageName);
		this.log.debug(DOCKER_VOLUME_HOST_KEY + " = " + this.hostVolume);
		this.log.debug(DOCKER_VOLUME_CONTAINER_KEY + " = " + this.containerVolume);
		this.log.debug(DOCKER_VOLUME_READONLY_KEY + " = " + this.readonly);
	}

	public String getImageName()
	{
		return this.imageName;
	}

	public String getHostVolume()
	{
		return this.hostVolume;
	}

	public String getContainerVolume()
	{
		return this.containerVolume;
	}

	public boolean isReadonly()
	{
		return this.readonly;
	}
}

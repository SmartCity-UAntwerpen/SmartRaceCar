package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class PortAspect extends Aspect
{
	private static final String PREFIX = "port";

	private static final String CLIENT_PORT_KEY = PREFIX + ".client";
	private static final String SERVER_PORT_KEY = PREFIX + ".server";

	private static final String[] KEYS = {CLIENT_PORT_KEY, SERVER_PORT_KEY};

	private static final String DEFAULT_SERVER_PORT = "5005";
	private static final String DEFAULT_CLIENT_PORT = "5006";


	private Logger log;

	private int clientPort;
	private int serverPort;

	public PortAspect(File configFile) throws IOException
	{
		super(AspectType.PORT);

		this.log = LoggerFactory.getLogger(PortAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.serverPort = Integer.parseInt(properties.getProperty(SERVER_PORT_KEY, DEFAULT_SERVER_PORT));
			this.clientPort = Integer.parseInt(properties.getProperty(CLIENT_PORT_KEY, DEFAULT_CLIENT_PORT));

			this.log.debug(SERVER_PORT_KEY + " = " + this.serverPort);
			this.log.debug(CLIENT_PORT_KEY + " = " + this.clientPort);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}

	}

	public int getClientPort()
	{
		return this.clientPort;
	}

	public int getServerPort()
	{
		return this.serverPort;
	}
}

package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class TcpServerAspect extends Aspect
{
	private static final String PREFIX = "TcpServer";

	private static final String SERVER_PORT_KEY = PREFIX + ".port";
	private static final String[] KEYS = {SERVER_PORT_KEY};

	private static final String DEFAULT_SERVER_PORT = "9999";

	private Logger log;
	private int serverPort;

	public TcpServerAspect (File configFile) throws IOException
	{
		super(AspectType.TCP_SERVER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.serverPort = Integer.parseInt(properties.getProperty(SERVER_PORT_KEY, DEFAULT_SERVER_PORT));

			this.log.debug(SERVER_PORT_KEY + " = " + this.serverPort);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public TcpServerAspect (int serverPort)
	{
		super(AspectType.TCP_SERVER);
		this.log = LoggerFactory.getLogger(RacecarAspect.class);

		this.serverPort = serverPort;
		this.log.debug(SERVER_PORT_KEY + " = " + this.serverPort);
	}

	public int getServerPort()
	{
		return this.serverPort;
	}
}

package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class TcpClientAspect extends Aspect
{
	private static final String PREFIX = "TcpClient";

	private static final String CLIENT_PORT_KEY = PREFIX + ".port";

	private static final String[] KEYS = {CLIENT_PORT_KEY};

	private static final String DEFAULT_CLIENT_PORT = "5006";


	private Logger log;

	private int clientPort;

	public TcpClientAspect(File configFile) throws IOException
	{
		super(AspectType.TCP_CLIENT);

		this.log = LoggerFactory.getLogger(TcpClientAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.clientPort = Integer.parseInt(properties.getProperty(CLIENT_PORT_KEY, DEFAULT_CLIENT_PORT));

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
}

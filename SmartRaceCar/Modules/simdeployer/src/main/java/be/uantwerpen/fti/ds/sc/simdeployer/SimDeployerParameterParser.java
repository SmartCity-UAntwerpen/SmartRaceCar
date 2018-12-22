package be.uantwerpen.fti.ds.sc.simdeployer;

import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;

public class SimDeployerParameterParser
{
	private static final String SERVER_PORT_KEY = "server.port";
	private static final String DOCKER_IMAGE_KEY = "Docker.image";

	private Logger log;

	public FileInputStream openFileStream(String file) throws IOException
	{
		try
		{
			String decodedPath = URLDecoder.decode(file, "UTF-8");
			return new FileInputStream(decodedPath);
		}
		catch (UnsupportedEncodingException uee)
		{
			// Catch, Log and re-throw
			this.log.warn("Could not decode file path.", uee);
			IOException ioe = new IOException(uee.getMessage());
			ioe.setStackTrace(uee.getStackTrace());
			throw ioe;
		}
		catch (IOException ioe)
		{
			// Catch, Log and re-throw
			this.log.warn("Could not open file", ioe);
			throw ioe;
		}
	}

	private SimDeployerParameters readParameters(String propertiesFile)
	{
		Properties prop = new Properties();
		InputStream input = null;

		try
		{
			input = this.openFileStream(propertiesFile);
			prop.load(input);
		}
		catch (IOException ioe)
		{
			this.log.warn("Could not open config file. Loading default settings.", ioe);
			return new SimDeployerParameters();
		}

		int serverPort = Integer.parseInt(prop.getProperty(SERVER_PORT_KEY));
		String dockerImage = prop.getProperty(DOCKER_IMAGE_KEY);

		this.log.info("SimDeployer config loaded.");

		SimDeployerParameters parameters = new SimDeployerParameters(serverPort, dockerImage);

		try
		{
			input.close();
		}
		catch (IOException ioe)
		{
			this.log.warn("Could not close config file.", ioe);
		}

		return parameters;
	}

	public SimDeployerParameterParser()
	{
		this.log = LoggerFactory.getLogger(SimDeployerParameterParser.class);
	}

	public SimDeployerParameters parse(String configurationFile)
	{
		return this.readParameters(configurationFile);
	}
}

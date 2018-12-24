package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;

public class Aspect
{
	private AspectType type;
	private Logger log;

	protected Properties openPropertiesFile(File file) throws IOException
	{
		try
		{
			String decodedPath = URLDecoder.decode(file.getAbsolutePath(), "UTF-8");
			FileInputStream fileStream = new FileInputStream(decodedPath);
			Properties properties = new Properties();
			properties.load(fileStream);
			return properties;
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

	protected Aspect (AspectType type)
	{
		this.log = LoggerFactory.getLogger(Aspect.class);
		this.type = type;
	}
}

package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;
import java.util.Set;

public class Aspect
{
	private AspectType type;
	private Logger log;

	protected void checkKeys (Properties properties, String[] keys)
	{
		Set<String> presentKeys = properties.stringPropertyNames();

		for (String key: keys)
		{
			if (!presentKeys.contains(key))
			{
				this.log.warn(key + " wasn't present in properties file, using default settings.");
			}
		}
	}

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

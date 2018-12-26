package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

public class KernelAspect extends Aspect
{
	private static final String PREFIX = "kernel";

	private static final String KERNEL_DEBUG_KEY = PREFIX + ".debug";
	private static final String[] KEYS = {KERNEL_DEBUG_KEY};

	private static final boolean DEFAULT_KERNEL_DEBUG = false;

	private Logger log;

	private boolean debug;

	public KernelAspect(File configFile) throws IOException
	{
		super(AspectType.KERNEL);

		this.log = LoggerFactory.getLogger(KernelAspect.class);

		try
		{
			Properties properties = this.openPropertiesFile(configFile);

			this.checkKeys(properties, KEYS);

			this.debug = Boolean.parseBoolean(properties.getProperty(KERNEL_DEBUG_KEY));

			this.log.debug(KERNEL_DEBUG_KEY + " = " + this.debug);
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to read " + this.getClass().getName() + " from \"" + configFile.getAbsolutePath() + "\"", ioe);
			throw ioe;
		}
	}

	public KernelAspect(boolean debug)
	{
		super(AspectType.KERNEL);
		this.log = LoggerFactory.getLogger(KernelAspect.class);
		this.debug = debug;
	}

	public boolean isDebug()
	{
		return this.debug;
	}
}

package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class Configuration
{
	private Logger log;
	private Map<AspectType, Aspect> aspects;

	public Configuration ()
	{
		this.log = LoggerFactory.getLogger(Configuration.class);
		this.aspects = new HashMap<>();
	}

	public void add(AspectType aspectType)
	{
		this.aspects.put(aspectType, null);
	}

	public void load(String configurationFilename)
	{
		File configFile = new File(configurationFilename);

		for (AspectType type: this.aspects.keySet())
		{
			try
			{
				switch (type)
				{
					case MQTT:
						this.aspects.put(type, new MqttAspect(configFile));
						continue;

					case ROS:
						this.aspects.put(type, new RosAspect(configFile));
						continue;
				}
			}
			catch (IOException ioe)
			{
				this.log.error("Failed to load configuration for aspect \"" + type + "\".", ioe);
			}
		}
	}
}

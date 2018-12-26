package be.uantwerpen.fti.ds.sc.common.configuration;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class Configuration
{
	private Logger log;
	private Map<AspectType, Aspect> aspects;

	public Configuration()
	{
		this.log = LoggerFactory.getLogger(Configuration.class);
		this.aspects = new HashMap<>();
	}

	public Configuration (Collection<AspectType> aspects)
	{
		for (AspectType type: aspects)
		{
			this.aspects.put(type, null);
		}
	}

	public void add(AspectType aspectType)
	{
		this.aspects.put(aspectType, null);
	}

	public Aspect get(AspectType aspectType)
	{
		return this.aspects.get(aspectType);
	}

	public Configuration load(String configurationFilename)
	{
		File configFile = new File(configurationFilename);

		for (AspectType type: this.aspects.keySet())
		{
			try
			{
				switch (type)
				{
					case BACKBONE:
						this.aspects.put(type, new BackboneAspect(configFile));
						break;

					case DOCKER:
						this.aspects.put(type, new DockerAspect(configFile));
						break;

					case MAP_MANAGER:
						this.aspects.put(type, new MapManagerAspect(configFile));
						break;

					case MQTT:
						this.aspects.put(type, new MqttAspect(configFile));
						break;

					case RACECAR:
						this.aspects.put(type, new RacecarAspect(configFile));
						break;

					case ROS:
						this.aspects.put(type, new RosAspect(configFile));
						break;

					case TCP_SERVER:
						this.aspects.put(type, new TcpServerAspect(configFile));
						break;

					case NAVSTACK:
						this.aspects.put(type, new NavStackAspect(configFile));
						break;

					default:
						this.log.warn("Ignoring unsupported aspect type: " + type);
						break;
				}
			}
			catch (IOException ioe)
			{
				this.log.error("Failed to load configuration for aspect \"" + type + "\".", ioe);
			}
		}

		return this;
	}
}

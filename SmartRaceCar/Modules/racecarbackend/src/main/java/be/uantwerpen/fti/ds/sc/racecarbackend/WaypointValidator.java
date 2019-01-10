package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.MqttMessages;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MapManagerAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

@Service
public class WaypointValidator implements MQTTListener
{
	private Logger log;
	private String currentMap;
	private MQTTUtils mqttUtils;
	private WaypointRepository waypointRepository;

	public WaypointValidator(@Qualifier("waypointValidator")Configuration configuration, @Autowired WaypointRepository waypointRepository)
	{
		this.log = LoggerFactory.getLogger(WaypointValidator.class);
		this.log.info("Initializing WaypointValidator...");

		this.waypointRepository = waypointRepository;

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to initialize MQTTUtils for WaypointValidator.", me);
		}

		MapManagerAspect mapManagerAspect = (MapManagerAspect) configuration.get(AspectType.MAP_MANAGER);
		this.currentMap = mapManagerAspect.getCurrentMap();

		this.log.info("Initialized WaypointValidator.");
	}

	public boolean exists(long id)
	{
		return this.waypointRepository.findByIdAndMapName(id, this.currentMap) != null;
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		// We only listen to map changes, the content of the message is the name of the new map
		this.currentMap = message;
	}
}

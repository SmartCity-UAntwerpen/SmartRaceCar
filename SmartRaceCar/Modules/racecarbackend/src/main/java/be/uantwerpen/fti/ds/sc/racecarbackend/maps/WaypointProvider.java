package be.uantwerpen.fti.ds.sc.racecarbackend.maps;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.MqttMessages;
import be.uantwerpen.fti.ds.sc.common.WayPoint;
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

import java.util.HashMap;
import java.util.Map;


@Service
public class WaypointProvider implements MQTTListener
{
	private Logger log;
	private Configuration configuration;

	private String currentMap;

	private MQTTUtils mqttUtils;

	private SqlWaypointRepository SQLRepository;
	private DebugWaypointRepository debugWaypointRepository;

	private boolean isMapChange(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);

		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
	}

	private boolean isDebug()
	{
		MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);
		return mapManagerAspect.isDatabaseDebug();
	}

	public WaypointProvider(@Qualifier("waypointProvider") Configuration configuration, @Autowired SqlWaypointRepository SQLRepository, @Autowired DebugWaypointRepository debugWaypointRepository)
	{
		this.log = LoggerFactory.getLogger(WaypointProvider.class);
		this.configuration = configuration;

		MapManagerAspect mapManagerAspect = (MapManagerAspect) this.configuration.get(AspectType.MAP_MANAGER);
		this.currentMap = mapManagerAspect.getCurrentMap();

		try
		{
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
		}
		catch (MqttException mqtte)
		{
			this.log.error("Failed to setup MQTTUtils for WayPointProvider ", mqtte);
		}

		this.SQLRepository = SQLRepository;
		this.debugWaypointRepository = debugWaypointRepository;
	}

	public Map<Long, WayPoint> getAll()
	{
		if(this.isDebug())
		{
			return this.debugWaypointRepository.loadWayPoints(this.currentMap);
		}
		else
		{
			Map<Long, WayPoint> wayPointMap = new HashMap<>();

			for(Waypoint waypoint: this.SQLRepository.findAllByMapName(this.currentMap))
			{
				wayPointMap.put(waypoint.getId(), new WayPoint(waypoint.getId(), waypoint.getX(), waypoint.getY(), waypoint.getZ(), waypoint.getW()));
			}
			return  wayPointMap;
		}
	}

	public WayPoint get(long id)
	{
		if(this.isDebug())
		{
			return this.debugWaypointRepository.loadWayPoints(this.currentMap).get(id);
		}
		else
		{
			Waypoint waypoint = this.SQLRepository.findByIdAndMapName(id, this.currentMap);
			return new WayPoint(waypoint.getId(), waypoint.getX(), waypoint.getY(), waypoint.getZ(), waypoint.getW());
		}
	}

	public boolean exists(long id)
	{
		if(this.isDebug())
		{
			return this.debugWaypointRepository.loadWayPoints(this.currentMap).containsKey(id);
		}
		else
		{
			return this.SQLRepository.existsByIdAndMapName(id, this.currentMap);
		}
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		if (this.isMapChange(topic))
		{
			this.log.info("Changing current map to \"" + message + "\".");
			this.currentMap = message;
		}
	}
}

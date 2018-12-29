package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SimDeployerCommunicator implements MQTTListener, SimDeployerCommunication
{
	private Logger log;
	private MQTTUtils mqttUtils;
	private long simID;

	private MessageListener listener;

	public SimDeployerCommunicator(Configuration configuration, long simID, MessageListener listener)
	{
		this.log = LoggerFactory.getLogger(SimDeployerCommunicator.class);

		MqttAspect aspect = (MqttAspect) configuration.get(AspectType.MQTT);

		try
		{
			this.mqttUtils = new MQTTUtils(aspect.getBroker(), aspect.getUsername(), aspect.getPassword(), this);
			this.mqttUtils.subscribe(aspect.getTopic() + "/simdeployer/#");
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MQTTUTils for SimDeployerCommunicator.", me);
		}

		this.simID = simID;
		this.listener = listener;
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		if(topic.endsWith(Long.toString(this.simID)))
		{
			// the message is meant for this vehicle

			String split[] = topic.split("/");

			String last = split[split.length-2];

			String m = JSONUtils.objectToJSONStringWithKeyWord(last, last);

			this.listener.notify(m);
		}
	}
}

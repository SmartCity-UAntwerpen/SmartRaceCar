package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.simkernel.SimkernelParameters;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class SimDeployerCommunicator implements MQTTListener, SimDeployerCommunication
{
	private Logger log;
	private SimkernelParameters params;
	private MQTTUtils mqttUtils;
	private long simID;

	private MessageListener listener;

	public SimDeployerCommunicator(SimkernelParameters params, long simID, MessageListener listener)
	{
		this.log = LoggerFactory.getLogger(SimDeployerCommunicator.class);
		this.params = params;

		try
		{
			this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
			this.mqttUtils.subscribe(this.params.getMqttTopic() + "/simdeployer");
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

			this.listener.notify(last);
		}
	}
}

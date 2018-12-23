package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.simkernel.SimkernelParameters;

public class SimDeployerCommunicator implements MQTTListener, SimDeployerCommunication
{
	private SimkernelParameters params;
	private MQTTUtils mqttUtils;
	private long simID;

	private MessageListener listener;

	public SimDeployerCommunicator(SimkernelParameters params, long simID, MessageListener listener)
	{
		this.params = params;
		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(this.params.getMqttTopic() + "/simdeployer");
		this.simID = simID;
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

package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.simkernel.SimkernelParameters;

public class SimDeployerCommunicator implements MQTTListener
{
	private SimkernelParameters params;
	private MQTTUtils mqttUtils;

	public SimDeployerCommunicator(SimkernelParameters params)
	{
		this.params = params;
		this.mqttUtils = new MQTTUtils(this.params.getMqttBroker(), this.params.getMqttUserName(), this.params.getMqttPassword(), this);
	}

	@Override
	public void parseMQTT(String topic, String message)
	{

	}
}

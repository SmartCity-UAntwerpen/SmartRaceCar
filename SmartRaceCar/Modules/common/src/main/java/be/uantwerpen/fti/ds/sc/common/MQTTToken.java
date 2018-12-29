package be.uantwerpen.fti.ds.sc.common;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttToken;

import java.io.IOException;

public class MQTTToken implements MessageToken
{
	private String message;
	private MqttToken token;

	public MQTTToken(String message, MqttToken token)
	{
		this.message = message;
		this.token = token;
	}

	@Override
	public String getMessage()
	{
		return this.message;
	}

	@Override
	public void waitForDelivery(long timeout) throws IOException
	{
		try
		{
			this.token.waitForCompletion(timeout);
		}
		catch (MqttException me)
		{
			IOException ioe = new IOException(me.getMessage(), me.getCause());
			throw ioe;
		}
	}
}

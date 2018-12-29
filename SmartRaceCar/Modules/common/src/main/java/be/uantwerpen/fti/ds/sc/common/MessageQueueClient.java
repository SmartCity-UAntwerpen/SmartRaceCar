package be.uantwerpen.fti.ds.sc.common;

public interface MessageQueueClient
{
	public void subscribe(String topic) throws Exception;

	public MessageToken publish(String topic, String message) throws Exception;
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

public interface MessageQueueClient
{
	public void subscribe(String topic);

	public void publish(String topic, String message);
}

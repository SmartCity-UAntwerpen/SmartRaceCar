package be.uantwerpen.fti.ds.sc.common;

import java.io.IOException;

public interface MessageToken
{
	public String getMessage();

	/**
	 * Block the calling thread until the message has been delivered.
	 * @param timeout       The maximum amount of time to wait, in seconds.
	 * @throws IOException
	 */
	public void waitForDelivery(long timeout) throws IOException;
}

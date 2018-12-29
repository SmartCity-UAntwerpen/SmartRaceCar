package be.uantwerpen.fti.ds.sc.common;

import java.io.IOException;

public interface MessageToken
{
	public String getMessage();

	public void waitForDelivery(long timeout) throws IOException;
}

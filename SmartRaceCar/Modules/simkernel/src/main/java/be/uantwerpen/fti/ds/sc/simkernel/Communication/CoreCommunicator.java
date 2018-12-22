package be.uantwerpen.fti.ds.sc.simkernel.Communication;

import be.uantwerpen.fti.ds.sc.common.TCPListener;
import be.uantwerpen.fti.ds.sc.common.TCPUtils;
import be.uantwerpen.fti.ds.sc.simkernel.SimkernelParameters;

import java.io.IOException;

public class CoreCommunicator implements TCPListener
{
	private SimkernelParameters params;
	private TCPUtils tcpUtils;

	public CoreCommunicator(SimkernelParameters params, int serverport, int clientport)
	{
		this.params = params;
		this.tcpUtils = new TCPUtils(serverport, clientport, this);
	}

	@Override
	public String parseTCP(String message) throws IOException
	{
		return null;
	}
}

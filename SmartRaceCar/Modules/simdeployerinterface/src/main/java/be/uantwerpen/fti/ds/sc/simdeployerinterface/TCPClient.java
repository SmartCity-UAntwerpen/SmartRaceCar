package be.uantwerpen.fti.ds.sc.simdeployerinterface;

import java.io.*;
import java.net.Socket;

public class TCPClient
{
	private static final int TIMEOUT = 30000;       // Socket timeout in milliseconds

	private String remoteHost;
	private int remotePort;
	private Socket socket;
	private BufferedWriter outStream;
	private BufferedReader inStream;

	public TCPClient (String remoteHost, int remotePort) throws IOException
	{
		this.remoteHost = remoteHost;
		this.remotePort = remotePort;
		this.socket = new Socket(remoteHost, remotePort);
		this.socket.setSoTimeout(TIMEOUT);
		this.outStream = new BufferedWriter(new OutputStreamWriter(this.socket.getOutputStream()));
		this.inStream = new BufferedReader(new InputStreamReader(this.socket.getInputStream()));
	}

	public void send(String data) throws IOException
	{
		this.outStream.write(data + '\n');
		this.outStream.newLine();
		this.outStream.flush();
	}

	public String receive() throws IOException
	{
		return this.inStream.readLine();
	}

	public void close() throws IOException
	{
		this.socket.close();
	}

	@Override
	public String toString()
	{
		return "TCP Client " + this.remoteHost + ":" + this.remotePort;
	}
}

package be.uantwerpen.fti.ds.sc.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * Help model to deal with TCP Socket communication. Can both be a listener and be used for specific messages.
 */
@SuppressWarnings("Duplicates")
public class TCPUtils extends Thread
{
	private Logger log;
	private ServerSocket serverSocket; // The server socket to be used for a one way connection for the listener.
	private Socket socket; // The duplex socket for a back and forth communication.
	private int clientPort; // Port to send messages to.
	private int serverPort; // Port to receive messages on.
	private BufferedReader in; // Buffered reader to read in the message.
	private PrintWriter out; // Writer to print the message to be send to.
	private TCPListener listener; // Interfaced listener to call methods in the classes that implement the interface.
	private boolean ackNack; // Boolean to see if the implementation of TCPUtils will be a duplex connection
	// with ACK/NACK responses(Used by Simdeployer to SimWorker communication)
	// or a simple listener(used by CORE to SimKernel/RosKernel communication).

	/**
	 * Help model to deal with TCP Socket communication. Can both be a listener and be used for specific messages.
	 *
	 * @param port     Port to listen and send messages on a duplex socket.
	 * @param listener Interface listener to trigger methods in classes implementing the TCP interface.
	 * @throws IOException
	 */
	public TCPUtils(int port, TCPListener listener) throws IOException
	{
		this.log = LoggerFactory.getLogger(TCPUtils.class);
		serverSocket = new ServerSocket(port);
		this.listener = listener;
		this.ackNack = true;
	}

	/**
	 * Help model to deal with TCP Socket communication. Can both be a listener and be used for specific messages.
	 *
	 * @param clientPort Port to send messages on over clientSocket.
	 * @param serverPort Port to listen for messages on over serverSocket.
	 * @param listener   Interface listener to trigger methods in classes implementing the TCP interface.
	 */
	public TCPUtils(int clientPort, int serverPort, TCPListener listener)
	{
		this.log = LoggerFactory.getLogger(TCPUtils.class);
		this.clientPort = clientPort;
		this.serverPort = serverPort;
		this.listener = listener;
		this.ackNack = false;
	}

	/**
	 * Help model to find a random open port. Used by SimDeployer to find available ports for simulated F1 cars
	 * to communicate over.
	 *
	 * @throws IOException
	 */
	public Integer findRandomOpenPort() throws IOException
	{
		try (
				ServerSocket socket = new ServerSocket(0)
		)
		{
			this.log.info("SOCKETS", "Port found:" + socket.getLocalPort());
			return socket.getLocalPort();

		}
	}

	/**
	 * Run the threaded TCP Socket Listener. If using the ACK/NACK variant it will also send a response. Otherwise
	 * it will trigger a callback on the listener Interface.
	 */
	public void run()
	{
		if (this.ackNack)
		{
			while (true)
			{
				try
				{
					Socket server = serverSocket.accept();

					try
					{
						this.in = new BufferedReader(new InputStreamReader(server.getInputStream()));
						this.out = new PrintWriter(server.getOutputStream(), true);
					} catch (IOException e)
					{
						this.log.warn("Cannot receive data." + e);
					}
					while (true)
					{
						try
						{
							//Send data back to client
							String data = in.readLine();
							this.log.info("Data received: " + data);
							String response = this.listener.parseTCP(data);
							this.out.println(response);
							this.log.info("Data Sent:" + response);
							break;
						} catch (IOException e)
						{
							this.log.error("Cannot receive data." + e);
						}
					}
					server.close();

				} catch (SocketTimeoutException s)
				{
					this.log.error("Timed out." + s);
				} catch (IOException e)
				{
					e.printStackTrace();
				}
			}
		} else
		{
			ServerSocket echoServer = null;
			String line;
			DataInputStream is;
			try
			{
				echoServer = new ServerSocket(serverPort);
			} catch (IOException e)
			{
				e.printStackTrace();
			}
			boolean run = true;
			while (run)
			{
				try
				{
					if (echoServer != null)
					{
						socket = echoServer.accept();
					}
					is = new DataInputStream(socket.getInputStream());

					line = is.readLine();
					if (line != null)
					{
						this.log.info("data received: " + line);
						listener.parseTCP(line);
					}
				} catch (IOException e)
				{
					this.log.error("Cannot receive data." + e);
				}
			}
			closeTCP();
		}
	}

	/**
	 * Send a message on the clientPort over a ClientSocket. Will continue to try until the message is received.
	 *
	 * @param data The message that has to be send.
	 */
	public void sendUpdate(String data)
	{
		Socket clientSocket = null;
		DataInputStream inputLine = new DataInputStream(new ByteArrayInputStream(data.getBytes(StandardCharsets.UTF_8)));
		byte[] bytes = new byte[100];
		Arrays.fill(bytes, (byte) 1);

		boolean connected = false;
		//if connection to socket can not be made, it waits until it can.
		while (!connected)
		{
			try
			{
				clientSocket = new Socket("localhost", clientPort);
				connected = true;
				try
				{
					PrintStream os = new PrintStream(clientSocket.getOutputStream());
					os.println(inputLine.readLine());
					this.log.info("SOCKETS", "Data Sent:" + data);
					os.close();
				} catch (UnknownHostException e)
				{
					this.log.warn("Could not send. Trying to connect to unknown host: " + e);
				} catch (IOException e)
				{
					this.log.error("Could not send. IOException:  " + e);
				}
			} catch (UnknownHostException e)
			{
				this.log.error("Cannot connect to receiver. Trying again." + e);
				connected = false;
			} catch (IOException e)
			{
				this.log.warn("Cannot connect to receiver to send   " + data + "   Trying again. Error:" + e);
				connected = false;
			}
		}
	}

	/**
	 * Send a message on a specified port over a ClientSocket. Will continue to try until the message is received.
	 *
	 * @param data The message that has to be send.
	 * @param port Port that the message has to be send over.
	 */
	public static void sendUpdate(String data, int port)
	{
		Logger log = LoggerFactory.getLogger(TCPUtils.class);
		Socket clientSocket = null;
		DataInputStream inputLine = new DataInputStream(new ByteArrayInputStream(data.getBytes(StandardCharsets.UTF_8)));
		byte[] bytes = new byte[100];
		Arrays.fill(bytes, (byte) 1);

		boolean connected = false;
		//if connection to socket can not be made, it waits until it can.
		int tryTimes = 0;
		while (!connected && tryTimes != 5)
		{
			try
			{
				clientSocket = new Socket("localhost", port);

				connected = true;
				try
				{
					PrintStream os = new PrintStream(clientSocket.getOutputStream());
					os.println(inputLine.readLine());
					log.info("Data Sent:" + data);
					os.close();
				} catch (UnknownHostException e)
				{
					log.warn("Could not send. Trying to connect to unknown host: " + e);
				} catch (IOException e)
				{
					log.error("Could not send. IOException:  " + e);
				}
			} catch (UnknownHostException e)
			{
				log.error("Cannot connect to receiver. Trying again." + e);
				connected = false;
			} catch (IOException e)
			{
				log.info("Cannot connect to receiver to send   " + data + "   Trying again(" + tryTimes + "/5). Error:" + e);
				connected = false;
				tryTimes++;
			}
		}
	}


	/**
	 * Close the connection.
	 */
	public void closeTCP()
	{
		try
		{
			socket.close();
		} catch (IOException e)
		{
			this.log.error("Could not close Socket connection. IOException:  " + e);
		} catch (Exception e)
		{
			this.log.error("Could not close Socket connection: ");
		}
	}
}
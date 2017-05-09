package be.uantwerpen.fti.ds.smartracecar.core;

import be.uantwerpen.fti.ds.smartracecar.common.JSONUtils;
import be.uantwerpen.fti.ds.smartracecar.common.Log;
import be.uantwerpen.fti.ds.smartracecar.common.Point;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

// Creates listener over sockets to listen to vehicle messages and sends back over other socket.
class TCPUtils extends Thread {

    private CoreListener listener;
    private Socket serverSocket;
    boolean run = true;
    private int clientPort;
    private int serverPort;

    TCPUtils(int clientPort, int serverPort, CoreListener listener){
        this.clientPort = clientPort;
        this.serverPort = serverPort;
        this.listener = listener;
    }

    //Listener thread
    public void run() {

        ServerSocket echoServer = null;
        String line;
        DataInputStream is;
        try {
            echoServer = new ServerSocket(serverPort);
        } catch (IOException e) {
            e.printStackTrace();
        }
        while (run) {
            try {
                if (echoServer != null) {
                    serverSocket = echoServer.accept();
                }
                is = new DataInputStream(serverSocket.getInputStream());
                line = is.readLine();
                if(line != null && JSONUtils.isJSONValid(line))
                    Log.logConfig("SOCKETS","data received: " + line);
                    //parses keyword to do the correct function call.
                    switch (JSONUtils.getFirst(line)) {
                        case "location":
                            listener.locationUpdate((Point) JSONUtils.getObject(line,Point.class));
                            break;
                        case "arrivedWaypoint":
                            listener.wayPointReached();
                            break;
                        case "connect":
                            listener.connectReceive();
                            break;
                        default:
                            Log.logWarning("SOCKETS","No matching keyword when parsing JSON. Data: " + line);
                            break;

                }

            } catch (IOException e) {
                Log.logSevere("SOCKETS","Cannot receive data." + e);
            }
        }
        closeTCP();
    }

    //sends message over clientSocket to vehicle
    void sendUpdate(String data) {
        Socket clientSocket = null;
        DataInputStream inputLine = new DataInputStream(new ByteArrayInputStream(data.getBytes(StandardCharsets.UTF_8)));
        byte[] bytes = new byte[100];
        Arrays.fill(bytes, (byte) 1);

        boolean connected = false;
        //if connection to socket can not be made, it waits until it can.
        while (!connected) {
            try {
                clientSocket = new Socket("localhost", clientPort);

                connected = true;
            } catch (UnknownHostException e) {
                Log.logSevere("SOCKETS","Cannot connect to car. Trying again." + e);
                connected = false;
            } catch (IOException e) {
                Log.logWarning("SOCKETS","Cannot connect to Car to send   " + data + "   Trying again. Error:" + e);
                connected = false;
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
        try {
            PrintStream os = new PrintStream(clientSocket.getOutputStream());
            os.println(inputLine.readLine());
            Log.logConfig("SOCKETS","Data Sent:" + data);
            os.close();
        } catch (UnknownHostException e) {
            Log.logWarning("SOCKETS","Could not send. Trying to connect to unknown host: " + e);
        } catch (IOException e) {
            Log.logSevere("SOCKETS","Could not send. IOException:  " + e);
        }
    }

    void closeTCP(){
        try {
            serverSocket.close();
        } catch (IOException e) {
            Log.logSevere("SOCKETS","Could not close Socket connection. IOException:  " + e);
        }
    }
}

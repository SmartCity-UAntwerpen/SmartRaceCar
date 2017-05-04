package SmartRacecar;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

class TCPUtils extends Thread {

    private CoreEvents listener;
    private Socket serverSocket;
    boolean run = true;
    private int clientPort;
    private int serverPort;

    TCPUtils(int clientPort, int serverPort, CoreEvents listener){
        this.clientPort = clientPort;
        this.serverPort = serverPort;
        this.listener = listener;
    }

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
                serverSocket = echoServer.accept();
                is = new DataInputStream(serverSocket.getInputStream());
                line = is.readLine();
                if(line != null && JSONUtils.isJSONValid(line))
                    Core.logConfig("SOCKETS","data received: " + line);
                    switch (JSONUtils.getFirst(line)) {
                        case "location":
                            listener.locationUpdate(0,0);
                            break;
                        case "arrivedWaypoint":
                            listener.updateRoute();
                            break;
                        case "connect":
                            listener.connectReceive();
                            break;
                        default:
                            Core.logWarning("SOCKETS","No matching keyword when parsing JSON. Data: " + line);
                            break;

                }

            } catch (IOException e) {
                Core.logSevere("SOCKETS","Cannot receive data." + e);
            }
        }
    }

    void sendUpdate(String data) {
        Socket clientSocket = null;
        DataInputStream inputLine = new DataInputStream(new ByteArrayInputStream(data.getBytes(StandardCharsets.UTF_8)));
        byte[] bytes = new byte[100];
        Arrays.fill(bytes, (byte) 1);

        boolean connected = false;
        while (!connected) {
            try {
                clientSocket = new Socket("localhost", clientPort);

                connected = true;
            } catch (UnknownHostException e) {
                Core.logSevere("SOCKETS","Cannot connect to car. Trying again." + e);
                connected = false;
            } catch (IOException e) {
                Core.logWarning("SOCKETS","Cannot connect to Car to send   " + data + "   Trying again. Error:" + e);
                connected = false;
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
        if (clientSocket != null) {
            try {
                PrintStream os = new PrintStream(clientSocket.getOutputStream());
                os.println(inputLine.readLine());
                Core.logConfig("SOCKETS","Data Sent:" + data);
                os.close();
            } catch (UnknownHostException e) {
                Core.logWarning("SOCKETS","Could not send. Trying to connect to unknown host: " + e);
            } catch (IOException e) {
                Core.logSevere("SOCKETS","Could not send. IOException:  " + e);
            }
        }
    }

    void closeTCP(){
        try {
            serverSocket.close();
        } catch (IOException e) {
            Core.logSevere("SOCKETS","Could not close Socket connection. IOException:  " + e);
        }
    }
}

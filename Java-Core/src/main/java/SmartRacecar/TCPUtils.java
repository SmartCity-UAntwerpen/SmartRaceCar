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

    private eventListener listener;
    private Socket serverSocket;
    boolean run;
    private int clientPort;
    private int serverPort;

    TCPUtils(int clientPort, int serverPort,eventListener listener){
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
                    System.out.println("[Sockets] [DEBUG] server received:" + line );
                    switch (JSONUtils.getFirst(line)) {
                        case "location":
                            listener.locationUpdate(0,0);
                            break;
                        case "nextWayPoint":
                            listener.updateRoute();
                            break;
                        case "arrivedWaypoint":
                            listener.updateRoute();
                            break;
                        default:
                            System.err.println("[Sockets] [DEBUG] No matching keyword when parsing JSON.");
                            break;

                }

            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] Cannot receive data." + e);
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
                System.err.println("[Sockets] [ERROR] " + e + ". Trying again.");
            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] Cannot connect to Car to send   " + data + "   Trying again. Error:" + e);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
        if (clientSocket != null) {
            try {
                PrintStream os = new PrintStream(clientSocket.getOutputStream());
                os.println(inputLine.readLine());
                System.out.println("[Sockets] [DEBUG] Data Send:" + data);
                os.close();
                listener.updateRoute(); //TODO remove test code
            } catch (UnknownHostException e) {
                System.err.println("[Sockets] [ERROR] Trying to connect to unknown host: " + e);
            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] IOException:  " + e);
            }
        }
    }

    void closeTCP(){
        try {
            serverSocket.close();
        } catch (IOException e) {
            System.err.println("[Sockets] [ERROR] IOException:  " + e);
        }
    }
}

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

public class TCPUtils extends Thread {

    eventListener listener;
    Socket serverSocket;
    public boolean run;
    int clientPort;
    int serverPort;

    public void addListener(eventListener listener) {
        this.listener = listener;
    }

    public TCPUtils(int clientPort, int serverPort){
        this.clientPort = clientPort;
        this.serverPort = serverPort;
    }

    public void run() {

        ServerSocket echoServer = null;
        String line;
        DataInputStream is;
        try {
            echoServer = new ServerSocket(serverPort);
        } catch (IOException e) {
            System.out.println(e);
        }
        while (run) {
            try {
                serverSocket = echoServer.accept();
                is = new DataInputStream(serverSocket.getInputStream());
                line = is.readLine();
                if(line != null && JSONUtils.isJSONValid(line))if(line != null){
                    System.out.println("[Sockets] [DEBUG] server received:" + line );
                    switch (JSONUtils.getFirst(line)) {
                        case "location":
                            listener.locationUpdate(0,0);
                            break;
                        case "job":
                            listener.jobRequest();
                            break;
                        default:
                            break;
                    }
                }

            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] Cannot receive data." + e);
            }
        }
    }

    public void sendUpdate(String data) {
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
                System.err.println("[Sockets] [ERROR] " + e + ". Trying again.");
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
                System.out.println("[Sockets] [DEBUG] Data Send:" + data.toString());
                os.close();
            } catch (UnknownHostException e) {
                System.err.println("[Sockets] [ERROR] Trying to connect to unknown host: " + e);
            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] IOException:  " + e);
            }
        }
    }

    public void closeTCP(){
        try {
            serverSocket.close();
        } catch (IOException e) {
            System.err.println("[Sockets] [ERROR] IOException:  " + e);
        }
    }
}

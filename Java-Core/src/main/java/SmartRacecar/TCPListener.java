package SmartRacecar;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class TCPListener extends Thread{

    private MyListener listener;
    Socket clientSocket;

    public void addListener(MyListener listener) {
        this.listener = listener;
    }

    public void run(){

        ServerSocket echoServer = null;
        String line;
        DataInputStream is;

        try {
            echoServer = new ServerSocket(5006);
        } catch (IOException e) {
            System.out.println(e);
        }
        try {
            clientSocket = echoServer.accept();
            is = new DataInputStream(clientSocket.getInputStream());

            while (true) {
                line = is.readLine();
                System.out.println("[Sockets] [DEBUG] server received:" + line );
                /*if(line != null && JSONUtils.isJSONValid(line))if(line != null){
                    System.out.println("[Sockets] [DEBUG] server received:" + line );
                    switch (JSONUtils.getFirst(line)) {
                        case "location":
                            System.out.println("[JSON] [DEBUG] it was a location.");
                            listener.locationUpdate();
                            break;
                        case "stopped":
                            System.out.println("[JSON] [DEBUG] it was a emergency stop.");
                            break;
                        default:
                            System.err.println("[JSON] [ERROR] Invalid keyword in JSON");
                            break;
                    }
                }*/
            }
        } catch (IOException e) {
            System.out.println(e);
        }
    }
}

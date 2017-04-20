package JavaCore;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.ServerSocket;

public class TCPListener extends Thread{
    public void run(){

        ServerSocket echoServer = null;
        String line;
        DataInputStream is;
        Socket clientSocket;

    /*
     * Open a server socket on port 5006. Note that we can't choose a port less
     * than 1023 if we are not privileged users (root).
     */
        try {
            echoServer = new ServerSocket(5006);
        } catch (IOException e) {
            System.out.println(e);
        }

    /*
     * Create a socket object from the ServerSocket to listen to and accept
     * connections. Open input and output streams.
     */
        try {
            clientSocket = echoServer.accept();
            is = new DataInputStream(clientSocket.getInputStream());

      /* As long as we receive data, echo that data back to the client. */
            while (true) {
                line = is.readLine();
                if(line != null)System.out.println("[Sockets] [DEBUG] server received:" + line );
            }
        } catch (IOException e) {
            System.out.println(e);
        }
    }

}

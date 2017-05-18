package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;
import java.net.*;
import java.io.*;

public class TCPUtils extends Thread {
    private ServerSocket serverSocket;
    private BufferedReader in;
    private PrintWriter out;
    private TCPListener listener;

    public TCPUtils(int port, TCPListener listener) throws IOException {
        serverSocket = new ServerSocket(port);
        this.listener = listener;
    }

    public void run() {
        while(true) {
            try {
                Socket server = serverSocket.accept();

                try {
                    in = new BufferedReader(new InputStreamReader(server.getInputStream()));
                    out = new PrintWriter(server.getOutputStream(),true);
                } catch (IOException e) {
                    Log.logSevere("SOCKETS","Cannot receive data." + e);
                }
                while(true){
                    try{
                        //Send data back to client
                        String data = in.readLine();
                        Log.logConfig("SOCKETS","data received: " + data);
                        String response = listener.parseTCP(data);
                        out.println(response);
                        Log.logConfig("SOCKETS","Data Sent:" + response);
                        break;
                    } catch (IOException e) {
                        Log.logSevere("SOCKETS","Cannot receive data." + e);
                    }
                }
                server.close();

            } catch (SocketTimeoutException s) {
                Log.logSevere("SOCKETS","Timed out." + s);
                break;
            } catch (IOException e) {
                e.printStackTrace();
                break;
            }
        }
    }
}
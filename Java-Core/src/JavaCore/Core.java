package JavaCore;

import org.json.simple.JSONObject;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

interface MyListener{
    void locationUpdate();
}

public class Core implements MyListener{

    double speed = 0;
    double rotation = 0;
    Socket clientSocket = null;
    DataInputStream inputLine = null;

    public Core() throws InterruptedException {
        TCPListener tcpListener = new TCPListener();
        tcpListener.start();
        tcpListener.addListener(this);

        while(true) {
            rotation = 20;
            sendWheelStates();
            Thread.sleep(1000);
            rotation = -20;
            sendWheelStates();
            Thread.sleep(1000);
            rotation = 0;
            speed = 1;
            sendWheelStates();
            Thread.sleep(1000);
            speed = 2;
            sendWheelStates();
            Thread.sleep(1000);
            speed = 0;
            sendWheelStates();
            Thread.sleep(1000);
        }
    }

    public void sendWheelStates(){
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("throttle",(int)speed);
        childData.put("steer",(int)rotation);
        parentData.put("location",childData);
        sendUpdate(parentData);
    }


    public void sendUpdate(JSONObject data){
        inputLine = new DataInputStream(new ByteArrayInputStream(data.toString().getBytes(StandardCharsets.UTF_8)));
        byte[] bytes = new byte[100];
        Arrays.fill(bytes,(byte)1);

        boolean connected = false;
        while(!connected) {
            try {
                clientSocket = new Socket("localhost", 5005);

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
                System.err.println("Trying to connect to unknown host: " + e);
            } catch (IOException e) {
                System.err.println("IOException:  " + e);
            }
        }
    }

    public void locationUpdate(){
        System.out.println("LOCATION UPDATED");
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        new Core();
    }
}
package JavaCore;

import org.jnativehook.GlobalScreen;
import org.jnativehook.NativeHookException;
import org.jnativehook.keyboard.NativeKeyEvent;
import org.jnativehook.keyboard.NativeKeyListener;
import org.json.simple.JSONObject;
import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.Logger;

public class InputController implements NativeKeyListener {

    private final Set<Integer> pressed = new HashSet<>(); // list of currently pressed buttons
    static int speed = 0; //speed of all wheels
    static int rotation = 0; //rotation of front wheels
    boolean driving = false; // state of driving
    static Socket clientSocket = null;
    static PrintStream os = null;
    DataInputStream inputLine = null;


    //action on when key is pressed down
    public void nativeKeyPressed(NativeKeyEvent e) {
        //stop program with ESC
        if (e.getKeyCode() == NativeKeyEvent.VC_ESCAPE) {
            os.close();
            try {
                clientSocket.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
            System.out.print("disconnected\n");
            try {
                GlobalScreen.unregisterNativeHook();
            } catch (NativeHookException e1) {
                e1.printStackTrace();
            }
        }
        //toggle driving state
        else if (e.getKeyCode() == NativeKeyEvent.VC_F4) {
            driving = !driving;
            System.out.println("driving:" + driving);
            if(!driving){
                speed = 0;
                rotation = 0;
            }
        }
        //safety stop of vehicle when SPACE is pressed
        else if (e.getKeyCode() == NativeKeyEvent.VC_SPACE) {
            speed = 0;
            rotation = 0;
            driving = false;
            System.out.println("SAFETY STOP");
        }else{
            if((!pressed.contains(e.getKeyCode()))) {
                pressed.add(e.getKeyCode()); //add key to keylist
                if(driving) try {
                    updateStatus();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
            }
        }
    }

    //action on when key is released
    public void nativeKeyReleased(NativeKeyEvent e) {
        pressed.remove(e.getKeyCode()); //remove key from keylist
        if(driving) try {
            updateStatus();
        } catch (IOException e1) {
            e1.printStackTrace();
        }
    }

    //update status of wheels based on keylist
    public void updateStatus() throws IOException {
        if(!pressed.contains(44) && !pressed.contains(31)) { // no Z or S
            speed = 0;
            rotation = 0;
        }
        if(pressed.contains(44)){ // Z
            if(pressed.contains(16)){ //Z & Q
                speed = 5;
                rotation = -50;
                System.out.println("forward left");
            }else if(pressed.contains(32)){ //Z & D
                speed = 5;
                rotation = 50;
                System.out.println("forward right");
            }else if(pressed.contains(31)){ //Z & S
                speed = 0;
                rotation = 0;
                System.out.println("forward backward");
            }else{ // only Z
                speed = 10;
                rotation = 0;
                System.out.println("forward");
            }
        }

        if(pressed.contains(31)){ // S
            if(pressed.contains(16)){ //S & Q
                speed = -5;
                rotation = -50;
                System.out.println("backward left");
            }else if(pressed.contains(32)){ //S & D
                speed = -5;
                rotation = 50;
                System.out.println("backward right");
            }else if(pressed.contains(44)){ //S & Z
                speed = 0;
                rotation = 0;
                System.out.println("backward forward");
            }else{ // only S
                speed = -10;
                rotation = 0;
                System.out.println("backward");
            }
        }
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("steer",rotation);
        childData.put("throttle", speed);
        parentData.put("drive",childData);
        sendUpdate(parentData);
    }

    public void sendUpdate(JSONObject json){
        inputLine = new DataInputStream(new ByteArrayInputStream((json.toString().getBytes(StandardCharsets.UTF_8))));
        if (clientSocket != null && os != null) {
            try {

        /*
         * Keep on reading from/to the socket till we receive the "Ok" from the
         * server, once we received that then we break.
         */
                os.println(inputLine.readLine());
        /*
         * Close the output stream, close the input stream, close the socket.
         */
            } catch (UnknownHostException e) {
                System.err.println("Trying to connect to unknown host: " + e);
            } catch (IOException e) {
                System.err.println("IOException:  " + e);
            }
        }
    }

    //action when key is typed in(virtual keyboards)
    public void nativeKeyTyped(NativeKeyEvent e) {
        System.out.println("Key Typed: " + e.getKeyText(e.getKeyCode()));
    }

    public static void main(String[] args) throws IOException {
        LogManager.getLogManager().reset();
        Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
        logger.setLevel(Level.OFF);
        try {
            GlobalScreen.registerNativeHook();
        }
        catch (NativeHookException ex) {

            System.exit(1);
        }
        TCPListener tcpListener = new TCPListener();
        tcpListener.start();

        try {
            clientSocket = new Socket("localhost", 5005);
            os = new PrintStream(clientSocket.getOutputStream());
        } catch (UnknownHostException e) {
            System.err.println(e);
        } catch (IOException e) {
            System.err.println(e);
        }

        GlobalScreen.addNativeKeyListener(new InputController());
    }
}
package JavaCore;

import org.json.simple.JSONObject;
import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Set;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class InputController{

    private final Set<String> pressed = new HashSet<>(); // list of currently pressed buttons
    double speed = 0; //speed of all wheels
    double leftTrigger = 0;
    double rightTrigger = 0;
    double rotation = 0; //rotation of front wheels
    boolean driving = false; // state of driving
    static Socket clientSocket = null;
    static PrintStream os = null;
    DataInputStream inputLine = null;
    public static String OS = null;

    public InputController() throws InterruptedException {
        TCPListener tcpListener = new TCPListener();
        tcpListener.start();
        boolean connected = false;
        while(!connected) {
            try {
                clientSocket = new Socket("localhost", 5005);
                os = new PrintStream(clientSocket.getOutputStream());
                connected = true;
                System.out.println("[Sockets] [DEBUG] Connected to vehicle.");
            } catch (UnknownHostException e) {
                System.err.println("[Sockets] [ERROR] " + e + ". Trying again.");
            } catch (IOException e) {
                System.err.println("[Sockets] [ERROR] " + e + ". Trying again.");
                Thread.sleep(2000);
            }
        }
        DeviceListener();
    }

    //update status of wheels based on keylist
    public void updateStatus() throws IOException {
        if(!pressed.contains("Z") && !pressed.contains("S")) { // no Z or S
            speed = 0;
            rotation = 0;
        }
        if(pressed.contains("Z")){ // Z
            if(pressed.contains("Q")){ //Z & Q
                speed = 1;
                rotation = -20;
                System.out.println("forward left");
            }else if(pressed.contains("D")){ //Z & D
                speed = 1;
                rotation = 20;
                System.out.println("forward right");
            }else if(pressed.contains("S")){ //Z & S
                speed = 0;
                rotation = 0;
                System.out.println("forward backward");
            }else{ // only Z
                speed = 2;
                rotation = 0;
                System.out.println("forward");
            }
        }

        if(pressed.contains("S")){ // S
            if(pressed.contains("Q")){ //S & Q
                speed = -1;
                rotation = -20;
                System.out.println("backward left");
            }else if(pressed.contains("D")){ //S & D
                speed = -1;
                rotation = 20;
                System.out.println("backward right");
            }else if(pressed.contains("Z")){ //S & Z
                speed = 0;
                rotation = 0;
                System.out.println("backward forward");
            }else{ // only S
                speed = -2;
                rotation = 0;
                System.out.println("backward");
            }
        }
    }

    public void sendUpdate(){
        JSONObject parentData = new JSONObject();
        JSONObject childData = new JSONObject();
        childData.put("steer",(int)rotation);
        childData.put("throttle",(int)speed);
        parentData.put("drive",childData);
        inputLine = new DataInputStream(new ByteArrayInputStream((parentData.toString().getBytes(StandardCharsets.UTF_8))));
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

    //Listener for input of devices
    public void DeviceListener(){
        System.out.println("[Controller] [DEBUG] Starting Driver...");
        Controller[] ca = ControllerEnvironment.getDefaultEnvironment().getControllers();
        System.out.println("[Controller] [DEBUG] Listing Devices...");
        //search for devices and create listeners for them.
        for(int j = 0;j<ca.length;j++){
            final Controller device = ca[j];

            System.out.println("Device #"+j+", name: "+ca[j].getName() + ", type: " + ca[j].getType()+ ", added.");
            Component[] components = ca[j].getComponents();
            for(int k = 0;k<components.length;k++){
                //run a new polling thread to listen for changes in components for each device.
                Thread t = new Thread() {
                    public void run() {
                        while(true){
                            try {
                                poll(device);
                            } catch (IOException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                };
                t.start();
            }
        }
    }

    //assistance method to get OS name
    public static String getOsName()
    {
        if(OS == null) { OS = System.getProperty("os.name"); }
        return OS;
    }

    //assistance method to check if OS is Windows
    public static boolean isWindows()
    {
        return getOsName().startsWith("Windows");
    }

    //this function will continuously listen for events in the device
    public void poll(Controller device) throws IOException {
        device.poll();
        EventQueue queue = device.getEventQueue();
        Event event = new Event();
        while(queue.getNextEvent(event)) {
            //get information about which component changes and it's value.
            Component comp = event.getComponent();
            float value = event.getValue();
            if(isWindows()){
                PollActionWindows(comp,value);
            }else{
                PollActionLinux(comp,value);
            }
        }
    }

    //this function checks which component has changed and does the appropriate action for that component in Windows
    public void PollActionWindows(Component comp, float value) throws IOException {

        //Xbox Controller Back button : exit
        if(comp.toString().equals("Button 6")){
            if(value==1.0f){
                os.close();
                try {
                    clientSocket.close();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
                System.exit(1);
            }
        }
        //Xbox Controller Start button: switch driving state
        else if(comp.toString().equals("Button 7")){
            if(value==1.0f){
                driving = !driving;
                System.out.println("driving:" + driving);
                if(!driving){
                    speed = 0;
                    rotation = 0;
                    sendUpdate();
                }
            }
        }
        //Xbox Controller Triggers: forward/backwards speed.
        else if(comp.toString().equals("Z Axis")){
            speed = Math.round((value*-1) * 100.0)  / 33;
            sendUpdate();
        }

        //Xbox Controller Right Analog Joystick: direction input
        else if(comp.toString().equals("X Axis")){
            double temp = rotation;
            rotation = Math.round(value * 100.0) / 4;
            if(rotation >= -3 && rotation <= 3)rotation = 0;
            if(Math.round(temp) != 0)sendUpdate();
        }
    }

    //this function checks which component has changed and does the appropriate action for that component in Linux
    public void PollActionLinux(Component comp, float value) throws IOException {

        if(comp.toString().equals("A") || comp.toString().equals("S") || comp.toString().equals("Q") || comp.toString().equals("D"))
            if(!pressed.contains(comp.toString())){
                pressed.add(comp.toString()); //add key to keylist
                updateStatus();
                sendUpdate();
            }
        //Keyboard F12: exit
        if(comp.toString().equals("F12")){
            os.close();
            try {
                clientSocket.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
            System.exit(1);
        }

        //Keyboard SPACE: switch driving state
        else if(comp.toString().equals("SPACE")){
            driving = !driving;
            System.out.println("driving:" + driving);
            if(!driving){
                speed = 0;
                rotation = 0;
                sendUpdate();
            }
        }

        //Xbox Controller Left Trigger
        else if(comp.toString().equals("rz")){
            double roundedvalue = Math.round((value*-1) * 100.0) / 33;
            double triggervalue = (roundedvalue+1)/2;
            leftTrigger = triggervalue;
            speed = rightTrigger - leftTrigger;
            sendUpdate();
        }
        //Xbox Controller Right Trigger
        else if(comp.toString().equals("z")){
            double roundedvalue = Math.round((value*-1) * 100.0) / 33;
            double triggervalue = (roundedvalue+1)/2;
            rightTrigger = triggervalue;
            speed = rightTrigger - leftTrigger;
            sendUpdate();
        }
        //Xbox Controller Right Analog Joystick: direction input
        else if(comp.toString().equals("x")){
            double temp = rotation;
            rotation = Math.round(value * 100.0) / 4;
            if(rotation <= -3 && rotation <= 3)rotation = 0;
            if(Math.round(temp) != 0)sendUpdate();
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        new InputController();
    }
}
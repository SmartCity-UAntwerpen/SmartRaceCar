package JavaCore;

import org.jnativehook.GlobalScreen;
import org.jnativehook.NativeHookException;
import org.jnativehook.keyboard.NativeKeyEvent;
import org.jnativehook.keyboard.NativeKeyListener;

import java.util.HashSet;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.Logger;

public class InputController implements NativeKeyListener {

    private final Set<Integer> pressed = new HashSet<>(); // list of currently pressed buttons
    int speed = 0; //speed of all wheels
    int rotation = 0; //rotation of front wheels
    boolean driving = false; // state of driving

    //action on when key is pressed down
    public void nativeKeyPressed(NativeKeyEvent e) {
        //stop program with ESC
        if (e.getKeyCode() == NativeKeyEvent.VC_ESCAPE) {
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
                if(driving)updateStatus();
            }
        }
    }

    //action on when key is released
    public void nativeKeyReleased(NativeKeyEvent e) {
        pressed.remove(e.getKeyCode()); //remove key from keylist
        if(driving)updateStatus();
    }

    //update status of wheels based on keylist
    public void updateStatus(){
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
        sendUpdate();
    }

    //send current status to car
    public void sendUpdate(){
        //TODO
    }

    //action when key is typed in(virtual keyboards)
    public void nativeKeyTyped(NativeKeyEvent e) {
        System.out.println("Key Typed: " + e.getKeyText(e.getKeyCode()));
    }

    public static void main(String[] args) {
        LogManager.getLogManager().reset();
        Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
        logger.setLevel(Level.OFF);
        try {
            GlobalScreen.registerNativeHook();
        }
        catch (NativeHookException ex) {

            System.exit(1);
        }

        GlobalScreen.addNativeKeyListener(new InputController());
    }
}
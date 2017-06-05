package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.smartracecar.common.TCPUtils;

import java.util.Arrays;

class SimulatedVehicle{

    private long startPoint = -1;
    private String name;
    private float speed = 0;
    private boolean deployed = false;
    private boolean available = false;
    private Simulation simulatedCore;
    private Simulation simulatedSimKernel;
    private int portOne;
    private int portTwo;

    SimulatedVehicle(long simulationID,String jarPath) {
        name = "SimCar" + simulationID;
        this.simulatedCore = new Simulation(jarPath + "\\Core.jar");
        this.simulatedSimKernel = new Simulation(jarPath + "\\SimKernel.jar");
    }

    void start(int portOne, int portTwo){
        this.portOne = portOne;
        this.portTwo = portTwo;
        simulatedCore.start(Arrays.asList(Long.toString(startPoint),Integer.toString(portOne),Integer.toString(portTwo)));
        simulatedSimKernel.start(Arrays.asList(Integer.toString(portTwo),Integer.toString(portOne)));
        this.deployed = true;
        this.available = true;

    }

    void stop() {
        TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("stop"), portOne);
        this.available = false;
    }

    void kill(){
        TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("kill"),portOne);
        simulatedCore.stop();
        simulatedSimKernel.stop();
    }

    void restart(){
        TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("restart"),portOne);
        this.available = true;
    }

    void run(){
        TCPUtils.sendUpdate(JSONUtils.keywordToJSONString("start"),portOne);
        this.available = true;
    }

    void ResetStartPoint(){
        TCPUtils.sendUpdate(JSONUtils.objectToJSONStringWithKeyWord("startpoint",startPoint),portOne);
    }

    long getStartPoint() {
        return startPoint;
    }

    void setName(String name) {
        this.name = name;
    }

    void setSpeed(float speed) {
        this.speed = speed;
    }

    void setStartPoint(long startPoint) {
        this.startPoint = startPoint;
    }

    boolean isDeployed() {
        return deployed;
    }

    public boolean isAvailable() {
        return available;
    }
}

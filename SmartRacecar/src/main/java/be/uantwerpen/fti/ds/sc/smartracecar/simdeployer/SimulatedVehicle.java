package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

/**
 * Created by Wouter Jansen on 5/15/2017.
 */
class SimulatedVehicle{

    private long startPoint = -1;
    private String name;
    private float speed = 0;
    private boolean deployed = false;

    SimulatedVehicle(long simulationID) {
        name = "SimCar" + simulationID;
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

    void setDeployed(boolean deployed) {
        this.deployed = deployed;
    }

    boolean isDeployed() {
        return deployed;
    }
}

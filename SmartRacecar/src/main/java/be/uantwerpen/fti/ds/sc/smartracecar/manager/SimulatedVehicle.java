package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

/**
 * Created by Wouter Jansen on 5/15/2017.
 */
public class SimulatedVehicle extends Vehicle{

    private long simulationID;
    private long startPoint;

    SimulatedVehicle(long ID, long simulationID, long startWayPoint, Point startLocation) {
        super(ID, startWayPoint, startLocation);
        this.simulationID = simulationID;
        this.startPoint = startWayPoint;
    }

    public long getStartPoint() {
        return startPoint;
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

/**
 * Created by Wouter Jansen on 5/15/2017.
 */
public class SimulatedVehicle extends Vehicle{

    private Long simulationID;

    SimulatedVehicle(Long ID, Long simulationID, int startWayPoint, Point startLocation) {
        super(ID, startWayPoint, startLocation);
        this.simulationID = simulationID;
    }
}

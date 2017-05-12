package be.uantwerpen.fti.ds.sc.smartracecar.manager;

/**
 * Created by Wouter Jansen on 5/12/2017.
 */
public class Location {

    private Long vehicleID;
    private int wayPointID;

    public Location(Long vehicleID, int wayPointID){
        this.vehicleID = vehicleID;
        this.wayPointID = wayPointID;
    }
}

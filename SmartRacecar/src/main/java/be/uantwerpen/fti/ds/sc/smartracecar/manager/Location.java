package be.uantwerpen.fti.ds.sc.smartracecar.manager;

class Location {

    private Long vehicleID;
    private int wayPointID;

    Location(Long vehicleID, int wayPointID){
        this.vehicleID = vehicleID;
        this.wayPointID = wayPointID;
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Location;

class Vehicle {

    private Long ID;
    private Location location;
    private boolean occupied = false;
    private boolean available = true;


    Vehicle(Long ID,long startWayPoint){
        this.ID = ID;
        location = new Location(ID,startWayPoint,startWayPoint,0);
    }

    void setOccupied(boolean state){
        occupied = state;
    }

    boolean getOccupied() {
        return occupied;
    }

    Long getID() {
        return ID;
    }

    public Location getLocation() {
        return location;
    }

    public void setLocation(Location location) {
        this.location = location;
    }

    public void setAvailable(boolean available) {
        this.available = available;
    }

    public boolean isAvailable() {
        return available;
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

class Vehicle {

    private Long ID;
    private long lastWayPoint;
    private Point location;
    private boolean occupied = false;



    Vehicle(Long ID,long startWayPoint,Point startLocation){
        this.ID = ID;
        this.lastWayPoint = startWayPoint;
        this.location = startLocation;
    }

    void setPoint(Point p){
        this.location = p;
    }

    void setLastWayPoint(long lastWayPoint){
        this.lastWayPoint = lastWayPoint;
    }

    long getLastWayPoint() {
        return lastWayPoint;
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
}

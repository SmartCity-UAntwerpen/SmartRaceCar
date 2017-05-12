package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

class Vehicle {

    private Long ID;
    private Point location;
    private boolean occupied = false;
    private int lastWayPoint;

    Vehicle(Long ID,int startWayPoint,Point startLocation){
        this.ID = ID;
        this.lastWayPoint = startWayPoint;
        this.location = startLocation;
    }

    void setPoint(Point p){
        this.location = p;
    }

    void setLastWayPoint(int lastWayPoint){
        this.lastWayPoint = lastWayPoint;
    }

    int getLastWayPoint() {
        return lastWayPoint;
    }

    void setOccupied(boolean state){
        occupied = state;
    }


    Point getLocation() {
        return location;
    }

    boolean getOccupied() {
        return occupied;
    }

    Long getID() {
        return ID;
    }
}

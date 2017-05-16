package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

class Vehicle {

    private Long ID;
    private long lastWayPoint;
    private Point location;
    private boolean occupied = false;
    private Float speed = Float.valueOf(3);



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


    Point getLocation() {
        return location;
    }

    boolean getOccupied() {
        return occupied;
    }

    Long getID() {
        return ID;
    }

    void setID(Long ID){
        this.ID = ID;
    }

    public void setLocation(Point location) {
        this.location = location;
    }

    public void setSpeed(Float speed) {
        this.speed = speed;
    }
}

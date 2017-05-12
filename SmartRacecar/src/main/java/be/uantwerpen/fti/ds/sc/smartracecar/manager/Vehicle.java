package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

public class Vehicle {

    private Long ID;
    private Point location;
    private boolean occupied = false;
    private int lastWayPoint;

    Vehicle(Long ID,int startWayPoint,Point startLocation){
        this.ID = ID;
        this.lastWayPoint = startWayPoint;
        this.location = startLocation;
    }

    public void setPoint(Point p){
        this.location = p;
    }

    public void setLastWayPoint(int lastWayPoint){
        this.lastWayPoint = lastWayPoint;
    }

    public int getLastWayPoint() {
        return lastWayPoint;
    }

    public void setOccupied(boolean state){
        occupied = state;
    }


    public Point getLocation() {
        return location;
    }

    public boolean getOccupied() {
        return occupied;
    }

    public Long getID() {
        return ID;
    }
}

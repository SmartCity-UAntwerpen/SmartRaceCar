package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

public class Vehicle {

    private int ID;
    private boolean simulated;
    private Point location;
    private boolean occupied = false;

    Vehicle(int ID,boolean simulated,Point startingLocation){
        this.ID = ID;
        this.simulated = simulated;
        this.location = startingLocation;
    }
}

package be.uantwerpen.fti.ds.model;

//Extended model of a Point which is used for waypoints. Supers the coordinates from the Point class.
public class WayPoint extends Point {

    private int ID = 0; // waypoint ID given by RaceCarManager.
    private int weight = 0; //weight used for pathfinding algorthims.

    public WayPoint(int ID,float x, float y,float z, float w, int weight) {
        super(x, y, z, w);
        this.ID = ID;
        this.weight = weight;
    }

    public int getID() {
        return ID;
    }
}

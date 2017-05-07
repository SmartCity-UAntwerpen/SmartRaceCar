package be.uantwerpen.fti.ds.model;

//model of drive parameters.
public class Drive {

    private float steer; //Rotation of the wheels.
    private float throttle; //speed of the vehicle's wheels.

    public Drive(float steer, float throttle){
        this.steer = steer;
        this.throttle = throttle;
    }
}

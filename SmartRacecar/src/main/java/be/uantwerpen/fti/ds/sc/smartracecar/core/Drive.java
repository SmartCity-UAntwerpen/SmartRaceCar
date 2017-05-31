package be.uantwerpen.fti.ds.sc.smartracecar.core;

//common of drive parameters.
class Drive {

    private float steer; //Rotation of the wheels.
    private float throttle; //speed of the vehicle's wheels.

    Drive(float steer, float throttle){
        this.steer = steer;
        this.throttle = throttle;
    }
}
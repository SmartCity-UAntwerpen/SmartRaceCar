package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Location;

import java.util.Date;

/**
 * Model that describes a F1 vehicle.
 */
public class Vehicle
{
    private Long ID; // ID of the vehicle.
    private Location location; // Location of the vehicle. Containing information on it's route progress.
    private boolean occupied = false; // If the vehicle is occupied by a current route job.
    private boolean available = true; // If the vehicle is available for jobs or other requests.
    private Job job;
    private Date heartbeat; //last known heartbeat of the vehicle


    /**
     * Model that describes a F1 vehicle.
     *
     * @param ID ID of the vehicle
     * @param startWayPoint ID of the waypoint where the vehicle starts.
     */
    public Vehicle(Long ID,long startWayPoint)
    {
        this.ID = ID;
        location = new Location(ID,startWayPoint,startWayPoint,100);
        heartbeat = new Date();
    }

    /**
     * Method to set the status if the vehicle is occupied by a route job already.
     *
     * @param state Status of occupation of teh vehicle by another route job.
     */
    void setOccupied(boolean state){
        occupied = state;
    }

    /**
     * Method to get the status if the vehicle is occupied by a route job already.
     *
     * @return Status of occupation of teh vehicle by another route job.
     */
    boolean getOccupied() {
        return occupied;
    }

    /**
     * Method to get the vehicle's ID.
     *
     * @return Long of the vehicle's ID.
     */
    Long getID() {
        return ID;
    }

    /**
     * Method to get the current location.
     *
     * @return Current location object.
     */
    Location getLocation() {
        return location;
    }

    /**
     * Method to set the current location.
     *
     * @param location Current location object.
     */
    void setLocation(Location location) {
        this.location = location;
    }

    /**
     * Method to set the status if the vehicle is available for jobs or other requests.
     *
     * @param available Status of availability of the vehicle
     */
    void setAvailable(boolean available) {
        this.available = available;
    }

    public Date getHeartbeat() {
        return heartbeat;
    }

    public void setHeartbeat(Date heartbeat) {
        this.heartbeat = heartbeat;
    }

    /**
     * Method to get the status if the vehicle is available for jobs or other requests.
     *
     * @return Status of availability of the vehicle
     */
    boolean isAvailable() {
        return available;
    }

    public Job getJob() {
        return job;
    }

    public void setJob(Job job) {
        this.job = job;
    }
}

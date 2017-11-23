package be.uantwerpen.fti.ds.sc.smartracecar.common;

/**
 * Model that describes a cost request.
 */
public class Cost {

    private boolean status; // Status describing the availabity of the vehicle.
    private int weightToStart; // Weight from current position to start position of route. In Seconds.
    private int weight; // Weight from start position to end position of route. In Seconds.
    private Long idVehicle; // ID of the vehicle being requested.

    /**
     * Model that describes a simulation of a Jar.
     *
     * @param status Status describing the availabity of the vehicle.
     * @param weightToStart Weight from current position to start position of route. In Seconds.
     * @param weight Weight from start position to end position of route. In Seconds.
     * @param idVehicle ID of the vehicle being requested.f
     */
    public Cost(boolean status, int weightToStart, int weight, Long idVehicle){
        this.status = status;
        this.weightToStart = weightToStart;
        this.weight = weight;
        this.idVehicle = idVehicle;
    }

    /**
     * Set the status of the vehicle being requested. Available means true, already on a route means false.
     *
     * @param status The status of the vehicle being requested. Available means true, already on a route means false.
     */
    public void setStatus(boolean status) {
        this.status = status;
    }

    /**
     * Set the ID of the vehicle being requested to calculate cost.
     *
     * @param idVehicle The ID of the vehicle being requested to calculate cost.
     */
    public void setIdVehicle(long idVehicle) {
        this.idVehicle = idVehicle;
    }

    /**
     * Get the weight between the current position and the start position of the route. In seconds.
     *
     * @return The weight between the current position and the start position of the route. In seconds.
     */
    public int getWeightToStart() {
        return weightToStart;
    }

    /**
     * Get the weight between the start position and the end position of the route. In seconds.
     *
     * @return The weight between the start position and the end position of the route. In seconds.
     */
    public int getWeight() {
        return weight;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Cost cost = (Cost) o;

        if (status != cost.status) return false;
        if (weightToStart != cost.weightToStart) return false;
        if (weight != cost.weight) return false;
        return idVehicle.equals(cost.idVehicle);
    }

}

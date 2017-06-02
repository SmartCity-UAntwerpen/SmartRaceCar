package be.uantwerpen.fti.ds.sc.smartracecar.common;

public class Cost {

    private boolean status;
    private int weightToStart;
    private int weight;
    private Long idVehicle;

    public Cost(boolean status, int weightToStart, int weight, Long idVehicle){
        this.status = status;
        this.weightToStart = weightToStart;
        this.weight = weight;
        this.idVehicle = idVehicle;
    }

    public void setStatus(boolean status) {
        this.status = status;
    }

    public void setIdVehicle(long idVehicle) {
        this.idVehicle = idVehicle;
    }

    public int getWeightToStart() {
        return weightToStart;
    }

    public int getWeight() {
        return weight;
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.common;

public class Cost {

    private boolean status;
    private Long weightToStart;
    private Long weight;
    private Long idVehicle;

    public Cost(boolean status, Long weightToStart, Long weight, Long idVehicle){
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

    public Long getWeightToStart() {
        return weightToStart;
    }

    public Long getWeight() {
        return weight;
    }
}

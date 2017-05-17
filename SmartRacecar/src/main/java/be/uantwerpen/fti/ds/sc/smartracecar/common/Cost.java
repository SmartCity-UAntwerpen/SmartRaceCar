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
}

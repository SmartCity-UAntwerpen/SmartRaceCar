package be.uantwerpen.fti.ds.sc.smartracecar.manager;

class Cost {

    private boolean status;
    private Long weightToStart;
    private Long weight;
    private Long idVehicle;

    Cost(boolean status, Long weightToStart, Long weight, Long idVehicle){
        this.status = status;
        this.weightToStart = weightToStart;
        this.weight = weight;
        this.idVehicle = idVehicle;
    }
}

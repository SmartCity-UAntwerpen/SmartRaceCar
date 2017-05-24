package be.uantwerpen.fti.ds.sc.smartracecar.common;

public class Location {

    private long idVehicle;
    private long idStart;
    private long idEnd;
    private int percentage;

    public Location(long idVehicle, long idStart, long idEnd, int percentage){
        this.idVehicle = idVehicle;
        this.idStart = idStart;
        this.idEnd  = idEnd;
        this.percentage = percentage;
    }

    public void setIdStart(long idStart) {
        this.idStart = idStart;
    }

    public void setIdEnd(long idEnd) {
        this.idEnd = idEnd;
    }

    public void setPercentage(int percentage) {
        this.percentage = percentage;
    }

    public long getIdEnd() {
        return idEnd;
    }

    public void setIdVehicle(long idVehicle) {
        this.idVehicle = idVehicle;
    }

    public int getPercentage() {
        return percentage;
    }
}

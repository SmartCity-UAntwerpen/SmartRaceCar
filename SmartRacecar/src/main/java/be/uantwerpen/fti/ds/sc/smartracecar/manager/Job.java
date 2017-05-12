package be.uantwerpen.fti.ds.sc.smartracecar.manager;

public class Job {

    private Long jobID;
    private Long idStart;
    private Long idEnd;
    private Long idVehicle;

    public Job(Long jobID,Long idStart, Long idEnd, Long idVehicle){
        this.jobID = jobID;
        this.idStart = idStart;
        this.idEnd = idEnd;
        this.idVehicle = idVehicle;
    }
}
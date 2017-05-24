package be.uantwerpen.fti.ds.sc.smartracecar.manager;

class Job {

    private Long idJob;
    private Long idStart;
    private Long idEnd;
    private Long idVehicle;

    public Job(Long idJob,Long idStart, Long idEnd, Long idVehicle){
        this.idJob = idJob;
        this.idStart = idStart;
        this.idEnd = idEnd;
        this.idVehicle = idVehicle;
    }

    public Long getIdJob() {
        return idJob;
    }

    public Long getIdStart() {
        return idStart;
    }

    public Long getIdEnd() {
        return idEnd;
    }

    public Long getIdVehicle() {
        return idVehicle;
    }
}
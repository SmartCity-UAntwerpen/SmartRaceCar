package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import javax.servlet.http.HttpServletResponse;
import javax.xml.ws.Response;
import java.util.Date;
import java.util.HashMap;

class HeartbeatChecker extends Thread {
    private RacecarBackend backend;

    public HeartbeatChecker(RacecarBackend backend)
    {
        this.backend=backend;
    }

    public void run() {
        try {
            Thread.sleep(30000); //Sleep for 30s
            HashMap<Long, Vehicle> vehicles = backend.getVehicles();
            Date currentTime = new Date();
            for (Long ID: vehicles.keySet()) {
                if((currentTime.getTime()-vehicles.get(ID).getHeartbeat().getTime())>30000) //longer than 30 seconds
                    backend.deleteVehicle(ID); //todo: logs schrijven en doorpipen
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;

import javax.servlet.http.HttpServletResponse;
import javax.xml.ws.Response;
import java.util.Date;
import java.util.HashMap;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
class HeartbeatChecker extends Thread {
    private RacecarBackend backend;

    /**
     * constructor for the HeartbeatChecker class
     * @param backend The backend object, where the vehicles are registered.
     */
    public HeartbeatChecker(RacecarBackend backend)
    {
        this.backend=backend;
    }

    /**
     * start the Checker thread
     */
    public void run() {
        while(true) {
            Log.logConfig("RACECAR_BACKEND", "Heartbeatchecker was started");
            try {
                Thread.sleep(30000); //Sleep for 30s
                Log.logConfig("RACECAR_BACKEND", "Heartbeats are being checked...");
                HashMap<Long, Vehicle> vehicles = backend.getVehicles();
                Date currentTime = new Date();
                for (Long ID : vehicles.keySet()) {
                    if ((currentTime.getTime() - vehicles.get(ID).getHeartbeat().getTime()) > 30000) //longer than 30 seconds
                    {
                        backend.deleteVehicle(ID);
                        Log.logWarning("RACECAR_BACKEND", "Vehicle with ID: " + ID + " was removed since it hasn't responded for over 30s");
                    }
                }
                Log.logConfig("RACECAR_BACKEND", "All heartbeats were checked.");

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

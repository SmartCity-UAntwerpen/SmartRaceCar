package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;
import be.uantwerpen.fti.ds.sc.smartracecar.common.RESTUtils;
import be.uantwerpen.fti.ds.sc.smartracecar.common.WayPoint;
import com.google.gson.reflect.TypeToken;

import javax.servlet.http.HttpServletResponse;
import javax.xml.ws.Response;
import java.lang.reflect.Type;
import java.util.Date;
import java.util.HashMap;

/**
 * Helper class to check the heartbeat of all registered vehicles periodically
 */
class HeartbeatChecker extends Thread {
    private RESTUtils restUtils;
    private String restURL = "http://smartcity.ddns.net:8081/carmanager"; // REST Service URL to RacecarBackend

    /**
     * constructor for the HeartbeatChecker class
     *
     * @param url RestURL of the backend object
     */
    public HeartbeatChecker(String url)
    {
        restUtils = new RESTUtils(restURL);
    }

    /**
     * start the Checker thread
     */
    public void run() {
        Log.logConfig("RACECAR_BACKEND", "Heartbeatchecker was started");
        while(true) {
            try {
                Thread.sleep(30000); //Sleep for 30s
                Log.logConfig("RACECAR_BACKEND", "Heartbeats are being checked...");
                Type typeOfHashMap = new TypeToken<HashMap<Long, Vehicle>>() {}.getType();
                HashMap<Long, Vehicle> vehicles = (HashMap<Long, Vehicle>) JSONUtils.getObjectWithKeyWord(restUtils.getJSON("getVehicles"), typeOfHashMap);
                Date currentTime = new Date();
                for (Long ID : vehicles.keySet()) {
                    if ((currentTime.getTime() - vehicles.get(ID).getHeartbeat().getTime()) > 90000) //longer than 90 seconds
                    {
                        restUtils.getCall("delete/" + ID);
                        Log.logWarning("RACECAR_BACKEND", "Vehicle with ID: " + ID + " was removed since it hasn't responded for over 90s");
                    }
                }
                Log.logConfig("RACECAR_BACKEND", "All heartbeats were checked.");

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}

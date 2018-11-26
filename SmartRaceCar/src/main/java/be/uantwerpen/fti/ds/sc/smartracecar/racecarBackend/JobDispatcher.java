package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.LogbackWrapper;

import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.Response;
import java.io.IOException;

public class JobDispatcher
{
    private LogbackWrapper log;
    private VehicleManager vehicleManager;

    public JobDispatcher(VehicleManager vehicleManager)
    {
        this.log = new LogbackWrapper();
        this.vehicleManager = vehicleManager;
    }

    @GET
    @Path("executeJob/{idJob}/{vehicleId}/{idStart}/{idEnd}")
    @Produces("text/plain")
    public Response jobRequest(@PathParam("idJob") long idJob, @PathParam("vehicleId") long vehicleId, @PathParam("idStart") long idStart, @PathParam("idEnd") long idEnd, String data, @Context HttpServletResponse response) throws IOException
    {
        Job job = new Job(idJob, idStart, idEnd, vehicleId);

        if (this.vehicleManager.exists(vehicleId))
        {

        }
        else
        {
            log.error();
        }
    }
}

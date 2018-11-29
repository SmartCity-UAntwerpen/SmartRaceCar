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
    private MapManager mapManager;
    private VehicleManager vehicleManager;

    public JobDispatcher(MapManager mapManager, VehicleManager vehicleManager)
    {
        this.log = new LogbackWrapper();
        this.mapManager = mapManager;
        this.vehicleManager = vehicleManager;
    }

    @GET
    @Path("executeJob/{jobId}/{vehicleId}/{startId}/{endId}")
    @Produces("text/plain")
    public Response jobRequest(@PathParam("jobId") long jobId, @PathParam("vehicleId") long vehicleId, @PathParam("startId") long startId, @PathParam("endId") long endId, String data, @Context HttpServletResponse response) throws IOException
    {
        Job job = new Job(jobId, startId, endId, vehicleId);

        if (!this.vehicleManager.exists(vehicleId))
        {
            String errorString = "Tried to execute job on non-existent vehicle (" + Long.toString(vehicleId) + ")";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_BAD_REQUEST, errorString);
            return Response.status(Response.Status.BAD_REQUEST).build();
        }

        if (this.vehicleManager.get(vehicleId).getOccupied())
        {
            String errorString = "Vehicle " + Long.toString(vehicleId) + " is currently occupied.";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        if (!this.vehicleManager.get(vehicleId).isAvailable())
        {
            String errorString = "Vehicle " + Long.toString(vehicleId) + " is currently not available.";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        if(!this.mapManager.exists(startId))
        {
            String errorString = "Request job with non-existent start waypoint " + Long.toString(startId) + ".";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        return Response.status(Response.Status.OK).build();
    }
}

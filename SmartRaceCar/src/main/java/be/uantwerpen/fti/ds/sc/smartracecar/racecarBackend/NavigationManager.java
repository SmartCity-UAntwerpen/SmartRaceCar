package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;
import com.google.gson.reflect.TypeToken;

import javax.servlet.http.HttpServletResponse;
import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.Produces;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.Response;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

// Route Update
// Cost Answers
public class NavigationManager implements MQTTListener
{
    private static class MQTTConstants
    {
        private static final Pattern COST_ANSWER_REGEX = Pattern.compile("racecar/[0-9]+/costanswer");
        private static final Pattern LOCATION_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/locationupdate");
    }

    private static final Type COST_TYPE = (new TypeToken<Cost>(){}).getType();

    private Parameters parameters;
    private LogbackWrapper log;
    private MQTTUtils mqttUtils;
    private List<Cost> costList;
    private VehicleManager vehicleManager;

    private boolean isCostAnswer(String topic)
    {
        Matcher matcher = MQTTConstants.COST_ANSWER_REGEX.matcher(topic);
        return matcher.matches();
    }

    private boolean isLocationUpdate(String topic)
    {
        Matcher matcher = MQTTConstants.LOCATION_UPDATE_REGEX.matcher(topic);
        return matcher.matches();
    }

    public NavigationManager(VehicleManager vehicleManager, Parameters parameters)
    {
        this.parameters = parameters;
        this.log = new LogbackWrapper();
        this.mqttUtils = new MQTTUtils(this.parameters.getMqttBroker(), this.parameters.getMqttUserName(), this.parameters.getMqttPassword(), this);
        this.mqttUtils.subscribeToTopic(this.parameters.getMqttTopic());
        this.costList = new ArrayList<>();
        this.vehicleManager = vehicleManager;
    }

    /**
     * REST GET server service to get a calculation cost of all available vehicles. It requests from each vehicle a calculation
     * of a possible route and returns a JSON containing all answers.
     *
     * @param startId Starting waypoint ID.
     * @param endId   Ending waypoint ID.
     * @return REST response of the type JSON containg all calculated costs of each vehicle.
     */
    @GET
    @Path("calcWeight/{startId}/{endId}")
    @Produces("application/json")
    public Response calculateCostsRequest(@PathParam("startId") long startId, @PathParam("endId") long endId, @Context HttpServletResponse response) throws IOException, InterruptedException
    {
        if (!wayPoints.containsKey(startId))
        {
            String errorString = "Request cost with non-existent start waypoint " + Long.toString(startId) + ".";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        if (!wayPoints.containsKey(endId))
        {
            String errorString = "Request cost with non-existent end waypoint " + Long.toString(endId) + ".";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        if (vehicles.isEmpty())
        {
            String errorString = "No vehicles exist" + Long.toString(endId) + ".";
            this.log.error("JOB-DISPATCHER", errorString);

            response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
            return Response.status(Response.Status.NOT_FOUND).build();
        }

        int totalVehicles = 0;
        int timer = 0;

        Iterator<Long> vehicleIdIterator = this.vehicleManager.getIdIterator();

        while (vehicleIdIterator.hasNext())
        {
            Long vehicleId = vehicleIdIterator.next();
            Vehicle vehicle = this.vehicleManager.get(vehicleId);

            if (vehicle.isAvailable())
            {
                totalVehicles++;
                this.mqttUtils.publishMessage("racecar/" + Long.toString(vehicle.getID()) + "/costrequest", Long.toString(startId) + " " + Long.toString(endId));
            }
        }

        // Runs for 100 iterations, each a little over 200ms
        while ((this.costList.size() < totalVehicles) && (timer != 100))
        {
            // Wait for each vehicle to complete the request or timeout after 100 attempts.
            Log.logInfo("RACECAR_BACKEND", "waiting for vehicles to complete request.");
            Thread.sleep(200);
            timer++;
        }

        ArrayList<Cost> costCopy = (ArrayList<Cost>) this.costList.clone();
        this.costList.clear();

        this.log.info("JOB-DISPATCHER", "Cost calculation request completed.");

        return Response.status(Response.Status.OK).entity(JSONUtils.arrayToJSONString(costCopy)).type("application/json").build();
    }

    /*
     *
     *      MQTT Parsing
     *
     */
    @Override
    public void parseMQTT(String topic, String message)
    {
        long id = TopicUtils.getCarId(topic);

        // id == -1 means the topic wasn't valid
        // It's also possible that the topic was valid, but the vehicle just doesn't exist
        if ((id != -1) && (this.vehicleManager.exists(id)))
        {
            // We received an MQTT cost answer
            if (this.isCostAnswer(topic))
            {
                Cost cost = (Cost)JSONUtils.getObject("value", COST_TYPE);
                this.costList.add(cost);
            }
            // We received an MQTT location update
            else if (this.isLocationUpdate(topic))
            {
                try
                {
                    long locationId = Long.parseLong(message);
                    int percentage = this.vehicleManager.get(id).getLocation().getPercentage();
                    Location location = new Location(id, locationId, locationId, percentage);
                    this.vehicleManager.get(id).setLocation(location);
                }
                catch (Exception vehicleNotFoundException)
                {
                    this.log.error("NAVIGATION-MAN", "Tried to update location on non-existent vehicle.");
                }
            }
        }
    }
}
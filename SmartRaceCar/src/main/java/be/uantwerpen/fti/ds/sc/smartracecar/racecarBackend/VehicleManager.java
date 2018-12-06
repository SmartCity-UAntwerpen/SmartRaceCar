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
import java.util.*;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Path("carmanager")
public class VehicleManager implements MQTTListener
{
	private static class MQTTConstants
	{
		private static final Pattern PERCENTAGE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/percentage");
		private static final Pattern AVAILABILITY_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/available");
		private static final Pattern ROUTE_UPDATE_REGEX = Pattern.compile("racecar/[0-9]+/route");
	}

	private static final Type LOCATION_TYPE = (new TypeToken<Location>()
	{
	}).getType();

	private BackendParameters parameters;
	private LogbackWrapper log;
	private MQTTUtils mqttUtils;
	private RESTUtils MaaSRestUtils;
	private RESTUtils backboneRestUtils;
	private NavigationManager navigationManager;
	private MapManager mapManager;                  // Non-Owning reference to Map Manager
	private HeartbeatChecker heartbeatChecker;
	private Map<Long, Vehicle> vehicles;

	private String getAvailabilityString(boolean available)
	{
		return available ? "Available" : "Not Available";
	}

	private boolean isPercentageUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.PERCENTAGE_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isAvailabilityUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.AVAILABILITY_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private boolean isRouteUpdate(String topic)
	{
		Matcher matcher = MQTTConstants.ROUTE_UPDATE_REGEX.matcher(topic);
		return matcher.matches();
	}

	private void updateRoute(long vehicleId, String mqttMessage)
	{
		switch (mqttMessage)
		{
			case "done":
				Vehicle vehicle = this.vehicles.get(vehicleId);
				vehicle.setOccupied(false);

				if (!this.parameters.isMaaSDisabled())
				{

					long jobId = vehicle.getJob().getIdJob();
					this.MaaSRestUtils.getTextPlain("completeJob/" + Long.toString(jobId));
				}

				vehicle.getLocation().setPercentage(100);

				log.info("VEHICLE-MAN", "Vehicle " + Long.toString(vehicleId) + " completed its route.");

				break;

			case "error":
				this.vehicles.get(vehicleId).setOccupied(false);
				log.info("VEHICLE-MAN", "Vehicle " + Long.toString(vehicleId) + " completed its route with errors.");
				break;

			case "notcomplete":
				this.vehicles.get(vehicleId).setOccupied(true);
				log.info("VEHICLE-MAN", "Vehicle " + Long.toString(vehicleId) + " hasn't completed its route yet.");
				break;
		}
	}

	public VehicleManager(BackendParameters parameters, MapManager mapManager)
	{
		this.parameters = parameters;
		this.log = new LogbackWrapper();

		this.log.info("VEHICLE-MAN", "Setting up MQTT...");

		this.mqttUtils = new MQTTUtils(this.parameters.getMqttBroker(), this.parameters.getMqttUserName(), this.parameters.getMqttPassword(), this);
		this.mqttUtils.subscribeToTopic(this.parameters.getMqttTopic());

		this.log.info("VEHICLE-MAN", "Setting up MaaS REST Utils...");

		this.MaaSRestUtils = new RESTUtils(parameters.getRESTCarmanagerURL());

		this.log.info("VEHICLE-MAN", "Setting up Backbone REST Utils...");

		this.backboneRestUtils = new RESTUtils(parameters.getBackboneRESTURL());

		this.log.info("VEHICLE-MAN", "Starting Navigation Manager...");

		this.navigationManager = new NavigationManager(this, this.mapManager, parameters);

		this.log.info("VEHICLE-MAN", "Setting Map Manager...");

		this.mapManager = mapManager;

		this.log.info("VEHICLE-MAN", "Creating Heartbeat Manager...");

		this.heartbeatChecker = new HeartbeatChecker(parameters.getRESTCarmanagerURL());
		this.vehicles = new HashMap<>();
	}

	public void start()
	{
		this.log.info("VEHICLE-MAN", "Starting Heartbeat Manager...");
		this.heartbeatChecker.start();
	}

	/**
	 * Checks whether or not a vehicle exists.
	 *
	 * @param vehicleId The id of the vehicle to be checked
	 * @return
	 */
	public boolean exists(long vehicleId)
	{
		return this.vehicles.containsKey(vehicleId);
	}

	/**
	 * Register a vehicle with the vehicle manager
	 *
	 * @param id
	 * @param vehicle
	 */
	public void register(long id, Vehicle vehicle)
	{
		this.vehicles.put(id, vehicle);
	}

	/**
	 * @param vehicleId
	 * @return
	 * @throws IndexOutOfBoundsException When a non-existent vehicle is queried, an exception is thrown
	 */
	public Vehicle get(long vehicleId)
	{
		if (this.exists(vehicleId))
		{
			return this.vehicles.get(vehicleId);
		} else
		{
			throw new IndexOutOfBoundsException("Tried to access vehicle that doesn't exist!");
		}
	}

	/**
	 * Returns an iterator over a keyset of the vehicle ID's.
	 *
	 * @return
	 */
	public Iterator<Long> getIdIterator()
	{
		return this.vehicles.keySet().iterator();
	}

	public int getNumVehicles()
	{
		return this.vehicles.size();
	}

	/*
	 *
	 *      REST Endpoints
	 *
	 */
	@GET
	@Path("delete/{id}")
	@Produces("text/plain")
	public Response delete(@PathParam("id") long id, @Context HttpServletResponse response) throws IOException
	{
		if (this.vehicles.containsKey(id))
		{
			if (!this.parameters.isBackboneDisabled())
			{
				this.backboneRestUtils.getTextPlain("bot/delete/" + Long.toString(id));
			}

			this.vehicles.remove(id);

			this.log.info("VEHICLE-MAN", "Removing vehicle " + Long.toString(id));

			return Response.status(Response.Status.OK).build();
		} else
		{
			String errorString = "Got delete request for vehicle " + Long.toString(id) + ", but vehicle doesn't exist.";
			this.log.warning("VEHICLE-MAN", errorString);
			response.sendError(HttpServletResponse.SC_NOT_FOUND, errorString);
		}

		return null;
	}

	@GET
	@Path("register/{startwaypoint}")
	@Produces("text/plain")
	public Response register(@PathParam("startwaypoint") long startwaypoint, @Context HttpServletResponse response) throws IOException
	{
		if (!this.mapManager.exists(startwaypoint))
		{
			String errorString = "Tried to register vehicle with non-existent start id.";
			this.log.error("VEHICLE-MAN", errorString);

			response.sendError(HttpServletResponse.SC_BAD_REQUEST, errorString);
			return Response.status(Response.Status.BAD_REQUEST).build();
		}

		long newVehicleId = -1;

		if (this.parameters.isBackboneDisabled())
		{
			newVehicleId = this.vehicles.size();
		} else
		{
			newVehicleId = Long.parseLong(this.backboneRestUtils.getJSON("bot/newBot/car"));
		}

		this.vehicles.put(newVehicleId, new Vehicle(newVehicleId, startwaypoint));

		this.log.info("VEHICLE-MAN", "Registered new vehicle (" + Long.toString(newVehicleId) + "), Current Waypoint: " + Long.toString(startwaypoint));

		return Response.status(Response.Status.OK).entity(newVehicleId).type("text/plain").build();
	}

	@GET
	@Path("posAll")
	@Produces("application/json")
	public Response getPositions(@Context HttpServletResponse response) throws IOException
	{
		List<Location> locations = new ArrayList<>();

		for (Long vehicleId : this.vehicles.keySet())
		{
			Vehicle vehicle = this.vehicles.get(vehicleId);
			locations.add(vehicle.getLocation());
		}

		this.log.info("VEHICLE-MAN", "Request for all positions processed, returning " + Integer.toString(locations.size()) + " locations.");

		return Response.status(Response.Status.OK).entity(JSONUtils.arrayToJSONString(locations)).type("application/json").build();
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

		if ((id != -1) && (this.exists(id)))
		{
			if (this.isPercentageUpdate(topic))
			{
				//todo: Refactor JSON message to only have percentage and no other location information
				Location location = (Location) JSONUtils.getObject(message, LOCATION_TYPE);
				this.vehicles.get(id).getLocation().setPercentage(location.getPercentage());
				log.info("VEHICLE-MAN", "Received Percentage update for vehicle " + Long.toString(id) + ", Status: " + Integer.toString(location.getPercentage()) + "%.");
			} else if (this.isAvailabilityUpdate(topic))
			{
				boolean availability = Boolean.parseBoolean(message);
				this.vehicles.get(id).setAvailable(availability);
				log.info("VEHICLE-MAN", "Received Availability update for vehicle " + Long.toString(id) + ", Status: " + this.getAvailabilityString(availability));
			} else if (this.isRouteUpdate(topic))
			{
				log.info("VEHICLE-MAN", "Received Route Update for vehicle " + Long.toString(id) + "");
				this.updateRoute(id, message);
			}
		}
	}
}

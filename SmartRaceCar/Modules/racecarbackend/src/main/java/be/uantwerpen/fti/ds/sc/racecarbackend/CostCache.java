package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.*;
import be.uantwerpen.fti.ds.sc.common.configuration.*;
import com.google.gson.reflect.TypeToken;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;

import javax.ws.rs.core.MediaType;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.*;
import java.util.Map;

@Controller
public class CostCache implements MQTTListener
{
	private Logger log;
	private Random random;
	private WaypointValidator waypointValidator;
	private CoordinateRepository coordinateRepository;
	private Configuration configuration;
	private MQTTUtils mqttUtils;
	private Map<Link, Float> costCache;

	private boolean isMapChange(String topic)
	{
		MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);

		return topic.startsWith(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
	}

	@Autowired
	public CostCache (@Qualifier("costCache") Configuration configuration, CoordinateRepository coordinateRepository, WaypointValidator waypointValidator)
	{
		this.log = LoggerFactory.getLogger(CostCache.class);
		this.random = new Random(System.currentTimeMillis());

		this.log.info("Initializing CostCache...");

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.mqttUtils.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.CHANGE_MAP);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MQTTUtils: ", me);
		}

		this.configuration = configuration;
		this.coordinateRepository = coordinateRepository;
		this.waypointValidator = waypointValidator;
		this.costCache = new HashMap<>();

		this.log.info("Initialized CostCache.");
	}

	public float calculateCost (long startId, long endId) throws IndexOutOfBoundsException, IOException
	{
		Link link = new Link(startId, endId);

		if (startId == endId)
		{
			return 0;
		}

		if (this.costCache.containsKey(link))
		{
			this.log.info("Got cache hit for link " + link);
			return this.costCache.get(link);
		}

		this.log.info("Got cache miss for link " + link);

		if (!this.waypointValidator.exists(startId))
		{
			String errorString = "Requested cost for start waypoint " + startId + ", but waypoint doesn't exist.";
			this.log.error(errorString);
			throw new IndexOutOfBoundsException(errorString);
		}

		if (!this.waypointValidator.exists(endId))
		{
			String errorString = "Requested cost for end waypoint " + endId + ", but waypoint doesn't exist.";
			this.log.error(errorString);
			throw new IndexOutOfBoundsException(errorString);
		}

		float cost = 0;

		RosAspect rosAspect = (RosAspect) this.configuration.get(AspectType.ROS);
		CostAspect costAspect = (CostAspect) this.configuration.get(AspectType.COST);
		if (!rosAspect.isRosDebug())
		{
			if ((costAspect.isIncreasingIds()) && (endId < startId))
			{
				cost = Float.MAX_VALUE;
			}
			else
			{
				Point startPointTmp = this.coordinateRepository.getCoordinates(startId);
				Point endPointTmp = this.coordinateRepository.getCoordinates(endId);

				Point startPoint = new Point(startPointTmp.getX(), startPointTmp.getY(), startPointTmp.getZ(), startPointTmp.getW());
				Point endPoint = new Point(endPointTmp.getX(), endPointTmp.getY(), endPointTmp.getZ(), endPointTmp.getW());

				List<Point> points = new ArrayList<>();
				points.add(startPoint);    // We need to add a Dummy point to the request, otherwise, the cost calculation server will return an error.
				points.add(startPoint);
				points.add(endPoint);

				String jsonString = JSONUtils.arrayToJSONString(points);
				String costString = "";

				try
				{
					RESTUtils ROSAPI = new RESTUtils(rosAspect.getRosServerUrl());
					Type costType = new TypeToken<Cost>(){}.getType();

					costString = ROSAPI.post(RESTMessages.RosServer.CALC_WEIGHT, jsonString, MediaType.APPLICATION_JSON_TYPE);
					Cost costObj = (Cost) JSONUtils.getObjectWithKeyWord(costString, costType);

					cost = costObj.getWeight();
				}
				catch (IOException ioe)
				{
					this.log.error("An exception was thrown while trying to calculate the cost for " + startId + " -> " + endId, ioe);
					throw ioe;
				}
				catch (NullPointerException npe)
				{
					this.log.error("Failed to extract Cost object from cost JSON (\"" + costString + "\")");
				}
			}
		}
		else
		{
			// Generate Random number in [0,100]
			// See: https://stackoverflow.com/a/363692

			cost = (this.random.nextFloat() * (costAspect.getUpperRange() - costAspect.getLowerRange())) + costAspect.getLowerRange();
		}

		this.costCache.put(link, cost);

		return cost;
	}

	/**
	 * REST Endpoint used to check the cost between the two points.
	 * The cost is calculated on the ROS navstack server (Usually at smartcity.ddns.net:8084)
	 * If no cars are available, a cost of infinity (Integer.MAX_VALUE) is returned
	 * @param startId
	 * @param endId
	 * @return
	 */
	@RequestMapping(value="/{startId}/{endId}", method=RequestMethod.GET, produces=MediaType.APPLICATION_JSON)
	public @ResponseBody ResponseEntity<String> costRequest(@PathVariable long startId, @PathVariable long endId)
	{
		Link link = new Link(startId, endId);

		this.log.info("Received cost request for " + link);

		try
		{
			float cost = this.calculateCost(startId, endId);
			String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", cost);
			return new ResponseEntity<>(responseJson, HttpStatus.OK);
		}
		catch (IndexOutOfBoundsException ioobe)
		{
			String errorString = "Cost calculation caused an IndexOutOfBoundsException: " + ioobe.getCause();
			this.log.error(errorString, ioobe);
			return new ResponseEntity<>(errorString, HttpStatus.BAD_REQUEST);
		}
		catch (IOException ioe)
		{
			String errorString = "Cost calculation caused an IOException: " + ioe.getCause();
			this.log.error(errorString, ioe);
			return new ResponseEntity<>(errorString, HttpStatus.SERVICE_UNAVAILABLE);
		}
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		if (this.isMapChange(topic))
		{
			this.log.info("Invalidating CostCache due to map name change. New map is \"" + message + "\".");
			this.costCache.clear();
		}
	}
}

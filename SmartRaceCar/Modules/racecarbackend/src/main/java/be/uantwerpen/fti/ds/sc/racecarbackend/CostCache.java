package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Cost;
import be.uantwerpen.fti.ds.sc.common.JSONUtils;
import be.uantwerpen.fti.ds.sc.common.Point;
import be.uantwerpen.fti.ds.sc.common.RESTUtils;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Controller
public class CostCache
{
	private Logger log;
	private WaypointValidator waypointValidator;
	private WaypointRepository waypointRepository;
	private CostCacheParameters costCacheParameters;
	private Map<Link, Integer> costCache;

	@Autowired
	public CostCache (CostCacheParameters costCacheParameters, WaypointRepository waypointRepository, WaypointValidator waypointValidator)
	{
		this.log = LoggerFactory.getLogger(CostCache.class);

		this.log.info("Initializing CostCache...");
		this.costCacheParameters = costCacheParameters;
		this.waypointRepository = waypointRepository;
		this.waypointValidator = waypointValidator;
		this.costCache = new HashMap<>();
		this.log.info("Initialized CostCache.");
	}

	public int calculateCost (long startId, long endId) throws IndexOutOfBoundsException
	{
		Link link = new Link(startId, endId);

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
			String errorString = "Requested cost for end waypoint " + startId + ", but waypoint doesn't exist.";
			this.log.error(errorString);
			throw new IndexOutOfBoundsException(errorString);
		}

		int cost = 0;

		if (!this.costCacheParameters.isROSServerDisabled())
		{
			RESTUtils ROSAPI = new RESTUtils(this.costCacheParameters.getROSServerURL());

			Type costType = new TypeToken<Cost>(){}.getType();

			Point startPointTmp = this.waypointRepository.getCoordinates(startId);
			Point endPointTmp = this.waypointRepository.getCoordinates(endId);

			Point startPoint = new Point(startPointTmp.getX(), startPointTmp.getY(), startPointTmp.getZ(), startPointTmp.getW());
			Point endPoint = new Point(endPointTmp.getX(), endPointTmp.getY(), endPointTmp.getZ(), endPointTmp.getW());

			List<Point> points = new ArrayList<>();
			points.add(startPoint);	// We need to add a Dummy point to the request, otherwise, the cost calculation server will return an error.
			points.add(startPoint);
			points.add(endPoint);

			String jsonString = JSONUtils.arrayToJSONString(points);

			try
			{
				String costString = ROSAPI.postJSONGetJSON("calcWeight", jsonString);
				Cost costObj = (Cost) JSONUtils.getObjectWithKeyWord(costString, costType);
				cost = costObj.getWeight();

				// We only cache if we're using the ROS Server
				// The whole point of caching was to take some load off the ROS Server,
				// so caching in case we don't use the ROS server is useless
				this.costCache.put(link, cost);
			}
			catch (IOException ioe)
			{
				this.log.error("An exception was thrown while trying to calculate the cost for " + startId + " -> " + endId, ioe);
				return Integer.MAX_VALUE;
			}
		}
		else
		{
			cost = 5;
		}

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
	@RequestMapping(value="/{startId}/{endId}", method= RequestMethod.GET, produces= MediaType.APPLICATION_JSON)
	public @ResponseBody
	ResponseEntity<String> costRequest(@PathVariable long startId, @PathVariable long endId)
	{
		Link link = new Link(startId, endId);

		this.log.info("Received cost request for " + link);

		try
		{
			int cost = this.calculateCost(startId, endId);
			String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", cost);
			return new ResponseEntity<>(responseJson, HttpStatus.OK);
		}
		catch (IndexOutOfBoundsException ioobe)
		{
			this.log.error("Cost calculation caused an exception: ", ioobe);
			return new ResponseEntity<>(ioobe.getMessage(), HttpStatus.BAD_REQUEST);
		}
	}
}

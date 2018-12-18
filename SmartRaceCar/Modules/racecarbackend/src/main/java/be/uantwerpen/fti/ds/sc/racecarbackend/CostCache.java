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
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Controller
public class CostCache
{
	private Logger log;
	private VehicleManager vehicleManager;
	private MapManager mapManager;
	private CostCacheParameters costCacheParameters;
	private Map<Link, Integer> costCache;

	@Autowired
	public CostCache (CostCacheParameters costCacheParameters, VehicleManager vehicleManager, MapManager mapManager)
	{
		this.log = LoggerFactory.getLogger(CostCache.class);

		this.log.info("Initializing CostCache...");
		this.costCacheParameters = costCacheParameters;
		this.vehicleManager = vehicleManager;
		this.mapManager = mapManager;
		this.costCache = new HashMap<>();
		this.log.info("Initialized CostCache.");
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

		if (this.costCache.containsKey(link))
		{
			this.log.info("Got cache hit for link " + link);
			String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", this.costCache.get(link));
			return new ResponseEntity<>(responseJson, HttpStatus.OK);
		}

		this.log.info("Got cache muss for link " + link);

		if (!this.mapManager.exists(startId))
		{
			this.log.error("Requested cost for start waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		if (!this.mapManager.exists(endId))
		{
			this.log.error("Requested cost for end waypoint " + startId + ", but waypoint doesn't exist.");
			return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
		}

		if (this.vehicleManager.getNumVehicles() == 0)
		{
			this.log.error("Requested cost, but no vehicles are available, returning " + Integer.MAX_VALUE);
			String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", Integer.MAX_VALUE);
			return new ResponseEntity<>(responseJson, HttpStatus.OK);
		}

		int cost = 0;

		if (!this.costCacheParameters.isROSServerDisabled())
		{
			RESTUtils ROSAPI = new RESTUtils(this.costCacheParameters.getROSServerURL());

			Type costType = new TypeToken<Cost>(){}.getType();

			Point startPointTmp = this.mapManager.getCoordinates(startId);
			Point endPointTmp = this.mapManager.getCoordinates(endId);

			Point startPoint = new Point(startPointTmp.getX(), startPointTmp.getY(), startPointTmp.getZ(), startPointTmp.getW());
			Point endPoint = new Point(endPointTmp.getX(), endPointTmp.getY(), endPointTmp.getZ(), endPointTmp.getW());

			List<Point> points = new ArrayList<>();
			points.add(startPoint);	// We need to add a Dummy point to the request, otherwise, the cost calculation server will return an error.
			points.add(startPoint);
			points.add(endPoint);

			String jsonString = JSONUtils.arrayToJSONString(points);
			String costString = ROSAPI.postJSONGetJSON("calcWeight", jsonString);
			Cost costObj = (Cost) JSONUtils.getObjectWithKeyWord(costString, costType);

			cost = costObj.getWeight();

			// We only cache if we're using the ROS Server
			// The whole point of caching was to take some load off the ROS Server,
			// so caching in case we don't use the ROS server is useless
			this.costCache.put(link, cost);
		}
		else
		{
			cost = 5;
		}

		String responseJson = JSONUtils.objectToJSONStringWithKeyWord("cost", cost);
		return new ResponseEntity<>(responseJson, HttpStatus.OK);
	}
}

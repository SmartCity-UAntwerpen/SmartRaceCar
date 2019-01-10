package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class WaypointValidator
{
	private WaypointRepository waypointRepository;

	public WaypointValidator(@Autowired WaypointRepository waypointRepository)
	{
		this.waypointRepository = waypointRepository;
	}

	public boolean exists(long id)
	{
		return this.waypointRepository.findById(id).isPresent();
	}
}

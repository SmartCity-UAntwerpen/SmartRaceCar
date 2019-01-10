package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.springframework.data.repository.CrudRepository;

public interface WaypointRepository extends CrudRepository<Waypoint, Long>
{
	public Waypoint findByIdAndMapName(long id, String mapName);

	public Iterable<Waypoint> findAllByMapName(String mapName);
}

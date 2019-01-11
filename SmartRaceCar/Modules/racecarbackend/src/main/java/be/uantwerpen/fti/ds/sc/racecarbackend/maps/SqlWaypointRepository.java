package be.uantwerpen.fti.ds.sc.racecarbackend.maps;

import org.springframework.data.repository.CrudRepository;

public interface SqlWaypointRepository extends CrudRepository<Waypoint, Long>
{
	public Waypoint findByIdAndMapName(long id, String mapName);

	public Iterable<Waypoint> findAllByMapName(String mapName);

	public boolean existsByIdAndMapName(long id, String mapName);
}

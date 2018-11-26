package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.WayPoint;

import java.util.Map;

public class MapManager
{
    private Map<Long, WayPoint> waypoints;

    public MapManager()
    {

    }

    public boolean exists(long id)
    {
        return waypoints.containsKey(id);
    }
}

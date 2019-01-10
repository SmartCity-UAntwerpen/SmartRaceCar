package be.uantwerpen.fti.ds.sc.core;

import java.util.LinkedList;
import java.util.Queue;

public class Job
{
	private Queue<Long> currentRoute;                                        // All waypoint IDs to be handled in the current route.
	private int routeSize;                                                   // Current route's size.
	private long jobID;

	public Job()
	{
		this.routeSize = -1;
		this.currentRoute = new LinkedList<>();
		this.jobID = -1;
	}

	public void add(long waypointID)
	{
		this.currentRoute.add(waypointID);
		this.routeSize = this.currentRoute.size();
	}

	public void setJobID(long jobID)
	{
		this.jobID = jobID;
	}

	public void clear()
	{
		this.currentRoute.clear();
		this.routeSize = -1;
	}

	public int getRouteSize()
	{
		return this.routeSize;
	}

	public int getRemainingRouteSize()
	{
		return this.currentRoute.size();
	}

	public long poll()
	{
		return this.currentRoute.poll();
	}

	public long peek()
	{
		return this.currentRoute.peek();
	}

	public boolean isEmpty()
	{
		return this.currentRoute.isEmpty();
	}

	public long getJobID()
	{
		return this.jobID;
	}
}

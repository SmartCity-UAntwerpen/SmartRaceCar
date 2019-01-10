package be.uantwerpen.fti.ds.sc.common;

public class TcpMessages
{
	/**
	 * TCP messages send by the Core
	 */
	public class Core
	{
		public static final String COST_TIMING = "costtiming";
		public static final String CONNECT = "connect";
		public static final String START_POINT = "startPoint";
		public static final String CURRENT_MAP = "currentMap";
		public static final String NEXT_WAYPOINT = "nextWayPoint";
		public static final String CURRENT_POSITION = "currentPosition";
		public static final String DRIVE = "drive";
	}

	/**
	 * TCP messages send by the Simkernel
	 */
	public class Simkernel
	{
		public static final String CONNECT = "connect";
		public static final String ARRIVED_WAYPOINT = "arrivedWaypoint";
		public static final String PERCENTAGE = "percentage";
		public static final String COST = "cost";
		public static final String COST_TIMING = "costtiming";
		public static final String EXIT = "exit";
	}
}

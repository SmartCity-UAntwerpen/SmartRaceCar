package be.uantwerpen.fti.ds.sc.common;

public class Messages
{
	/***
	 * Messages originating from Core
	 */
	public class CORE
	{
		public static final String COST = "cost";
		public static final String COST_TIMING = "costtiming";
		public static final String CONNECT = "connect";
		public static final String START_POINT = "startPoint";
		public static final String CURRENT_MAP = "currentMap";
		public static final String NEXT_WAYPOINT = "nextWayPoint";
		public static final String CURRENT_POSITION = "currentPosition";
	}

	/***
	 * Messages originating from SimDeployer
	 */
	public class SIMDEPLOYER
	{
		public static final String KILL = "kill";
	}

	/***
	 * Messages originating from SimKernel
	 */
	public class SIMKERNEL
	{
		public static final String CONNECT = "connect";
		public static final String ARRIVED_WAYPOINT = "arrivedWaypoint";
		public static final String PERCENTAGE = "percentage";
		public static final String COST = "cost";
		public static final String COST_TIMING = "costtiming";
		public static final String EXIT = "exit";
	}
}

package be.uantwerpen.fti.ds.sc.common;

public class MqttMessages
{
	public class Topics
	{
		/**
		 * Messages originating from Backend
		 */
		public class Backend
		{
			public static final String CHANGE_MAP = "changemap";
			public static final String JOB = "job";
			public static final String REGISTRATION_DONE = "registered";

			public static final String REGISTER = "register";
			public static final String DELETE = "delete";
		}

		public class Core
		{
			public static final String HEARTBEAT = "heartbeat";
			public static final String PERCENTAGE = "percentage";
			public static final String LOCATION_UPDATE = "locationupdate";
			public static final String ROUTE = "route";
		}

		public class SimDeployer
		{
			public static final String KILL = "kill";
		}
	}

	public class Messages
	{
		public class Core
		{
			public static final String NOT_COMPLETE = "notcomplete";
			public static final String ERROR = "error";
			public static final String DONE = "done";
			public static final String HEARTBEAT = "heartbeat";
		}

		public class SimDeployer
		{
			public static final String KILL = "kill";
		}
	}


}

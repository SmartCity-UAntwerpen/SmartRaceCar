package be.uantwerpen.fti.ds.sc.common;

public class MqttMessages
{
	public class Topics
	{
		/**
		 * Topics of messages originating from Backend
		 */
		public class Backend
		{
			public static final String CHANGE_MAP = "changemap";
			public static final String JOB = "job";
			public static final String REGISTRATION_DONE = "registered";

			public static final String REGISTER = "register";
			public static final String DELETE = "delete";
		}

		/**
		 * Topics of messages originating from Core
		 */
		public class Core
		{
			public static final String HEARTBEAT = "heartbeat";
			public static final String PERCENTAGE = "percentage";
			public static final String LOCATION_UPDATE = "locationupdate";
			public static final String ROUTE = "route";
		}

		/**
		 * Topics of messages originating from SimDeployer
		 */
		public class SimDeployer
		{
			public static final String KILL = "kill";
		}
	}

	public class Messages
	{
		/**
		 * Messages originating from Core
		 */
		public class Core
		{
			public static final String NOT_COMPLETE = "notcomplete";
			public static final String ERROR = "error";
			public static final String DONE = "done";
			public static final String HEARTBEAT = "heartbeat";
		}

		/**
		 * Messages originating from SimDeployer
		 */
		public class SimDeployer
		{
			public static final String KILL = "kill";
		}
	}


}

package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import org.glassfish.jersey.server.ResourceConfig;

/**
 * Configuration class of the REST Jerser service. Defining the main class where the REST interface will be found.
 */
class JerseyConfiguration extends ResourceConfig
{

	JerseyConfiguration()
	{
		packages("be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend");
	}
}

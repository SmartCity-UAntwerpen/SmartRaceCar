package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

/*
import org.apache.catalina.Context;
import org.apache.catalina.startup.Tomcat;
import org.glassfish.jersey.server.ResourceConfig;
import org.glassfish.jersey.servlet.ServletContainer;
*/
/**
 * Configuration class defining the build in TomCat Service.
 */
class TomCatLauncher
{

	/**
	 * run the launcher
	 *
	 * @throws Exception
	 *
	void start() throws Exception
	{

		String contextPath = "";
		String appBase = ".";
		String port = System.getenv("PORT");
		if (port == null || port.isEmpty())
		{
			port = "8081";
		}

		Tomcat tomcat = new Tomcat();
		tomcat.setPort(Integer.valueOf(port));
		tomcat.getHost().setAppBase(appBase);
		Context context = tomcat.addWebapp(contextPath, appBase);

		Tomcat.addServlet(context, "jersey-container-servlet", new ServletContainer(resourceConfig()));
		context.addServletMapping("/*", "jersey-container-servlet");

		tomcat.start();
		tomcat.getServer().await();
	}

	private ResourceConfig resourceConfig()
	{
		return new JerseyConfiguration();
	}
	*/
}

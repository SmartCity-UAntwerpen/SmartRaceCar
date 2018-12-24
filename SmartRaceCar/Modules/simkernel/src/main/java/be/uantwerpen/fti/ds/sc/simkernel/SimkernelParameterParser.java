package be.uantwerpen.fti.ds.sc.simkernel;

import be.uantwerpen.fti.ds.sc.common.ParameterParser;
import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class SimkernelParameterParser extends ParameterParser
{
    private static final String ROS_DEBUG_KEY = "ROS.debug";
    private static final String ROS_SERVER_URL_KEY = "ROS.URL";

    private Logger log;

    public SimkernelParameterParser()
    {
        this.log = LoggerFactory.getLogger(SimkernelParameters.class);
    }

    public SimkernelParameters parse(String propertiesFile)
    {
        Parameters parameters = super.parse(propertiesFile);

        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            ParameterParser parser = new ParameterParser();
            input = parser.openFileStream(propertiesFile);
            prop.load(input);
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not open config file. Loading default settings.", ioe);
            return new SimkernelParameters(parameters, true, "http://smartcity.ddns.net:8084");
        }

        boolean debugWithoutROS = Boolean.parseBoolean(prop.getProperty(ROS_DEBUG_KEY));

        String rosServerUrl = prop.getProperty(ROS_SERVER_URL_KEY);

        this.log.info("Backend config loaded.");

        SimkernelParameters simkernelParameters = new SimkernelParameters(parameters, debugWithoutROS, rosServerUrl);

        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not close config file. Loading default settings.", ioe);
        }

        return simkernelParameters;
    }
}

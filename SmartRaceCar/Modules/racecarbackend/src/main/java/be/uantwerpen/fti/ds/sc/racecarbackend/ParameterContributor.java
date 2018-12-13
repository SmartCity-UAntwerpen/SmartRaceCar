package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.ParameterParser;
import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

@Configuration
public class ParameterContributor
{
    // MAAS KEYS
    private static final String MAAS_DEBUG_KEY = "MaaS.debug";
    private static final String MAAS_URL_KEY = "MaaS.URL";

    // BACKBONE KEYS
    private static final String BACKBONE_DEBUG_KEY = "Backbone.debug";
    private static final String BACKBONE_URL_KEY = "Backbone.URL";

    // MAP MANAGER KEYS
    private static final String MAP_PATH_KEY = "Maps.path";
    private static final String MAP_CURRENT_KEY = "Maps.current";

    @Value("${Racecar.config_file}")
    private String DEFAULT_PROPERTIES_FILE;

    private Logger log;

    private BackendParameters readBackendParameters(String propertiesFile)
    {
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
            return new BackendParameters();
        }

        boolean debugWithoutMAAS = Boolean.parseBoolean(prop.getProperty(MAAS_DEBUG_KEY));
        boolean debugWithoutBackBone = Boolean.parseBoolean(prop.getProperty(BACKBONE_DEBUG_KEY));

        String restURLMAAS = prop.getProperty(MAAS_URL_KEY);
        String restURLBackBone = prop.getProperty(BACKBONE_URL_KEY);

        this.log.info("Backend config loaded.");

        BackendParameters backendParameters = new BackendParameters(this.parameters(), debugWithoutMAAS, debugWithoutBackBone, restURLMAAS, restURLBackBone);

        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not close config file. Loading default settings.", ioe);
        }

        return backendParameters;
    }

    private MapManagerParameters readMapManagerParameters(String propertiesFile)
    {
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
            return new MapManagerParameters();
        }

        String mapPath = prop.getProperty(MAP_PATH_KEY);
        String currentMap = prop.getProperty(MAP_CURRENT_KEY);

        this.log.info("Map Manager config loaded.");

        MapManagerParameters mapManagerParameters = new MapManagerParameters(this.readBackendParameters(propertiesFile), currentMap, mapPath);
        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not close config file. Loading default settings.", ioe);
        }

        return mapManagerParameters;
    }

    public ParameterContributor()
    {
        this.log = LoggerFactory.getLogger(ParameterContributor.class);
    }

    @Bean
    Parameters parameters()
    {
        ParameterParser parser = new ParameterParser();

        return parser.parse(this.DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("backend")
    BackendParameters backendParameters()
    {
        return this.readBackendParameters(this.DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("mapmanager")
    MapManagerParameters mapManagerParameters()
    {
        return this.readMapManagerParameters(this.DEFAULT_PROPERTIES_FILE);
    }
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.PropertySource;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;

@Configuration
public class ParameterConfig
{
    // MQTT PARAMETER KEYS
    private static final String MQTT_BROKER_KEY = "mqtt.broker";
    private static final String MQTT_USERNAME_KEY = "mqtt.username";
    private static final String MQTT_PASSWORD_KEY = "mqtt.password";
    private static final String MQTT_TOPIC_KEY = "mqtt.topic";

    // MAAS KEYS
    private static final String MAAS_DEBUG_KEY = "MaaS.debug";
    private static final String MAAS_URL_KEY = "MaaS.URL";

    // BACKBONE KEYS
    private static final String BACKBONE_DEBUG_KEY = "Backbone.debug";
    private static final String BACKBONE_URL_KEY = "Backbone.URL";

    // RACECAR KEYS
    private static final String RACECAR_URL_KEY = "Racecar.URL";

    // MAP MANAGER KEYS
    private static final String MAP_PATH_KEY = "Maps.path";
    private static final String MAP_CURRENT_KEY = "Maps.current";

    private Logger log;

    @Value("${Racecar.config_file}")
    private String DEFAULT_PROPERTIES_FILE;

    private FileInputStream openFileStream(String file) throws IOException
    {
        try
        {
            String decodedPath = URLDecoder.decode(file, "UTF-8");
            return new FileInputStream(decodedPath);
        }
        catch (UnsupportedEncodingException uee)
        {
            // Catch, Log and re-throw
            this.log.warn("Could not decode file path.", uee);
            IOException ioe = new IOException(uee.getMessage());
            ioe.setStackTrace(uee.getStackTrace());
            throw ioe;
        }
        catch (IOException ioe)
        {
            // Catch, Log and re-throw
            this.log.warn("Could not open file", ioe);
            throw ioe;
        }
    }

    private Parameters readParameters(String propertiesFile)
    {
        Properties prop = new Properties();
        InputStream  input = null;

        try
        {
            input = this.openFileStream(propertiesFile);
            prop.load(input);
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not open config file. Loading default settings.", ioe);
            return new Parameters();
        }

        String mqttBroker = prop.getProperty(MQTT_BROKER_KEY);
        String mqttUsername = prop.getProperty(MQTT_USERNAME_KEY);
        String mqttPassword = prop.getProperty(MQTT_PASSWORD_KEY);
        String mqttTopic = prop.getProperty(MQTT_TOPIC_KEY);
        String restCarmanagerURL = prop.getProperty(RACECAR_URL_KEY);

        this.log.info("Standard config loaded.");

        Parameters parameters = new Parameters(mqttBroker, mqttUsername, mqttPassword, restCarmanagerURL, mqttTopic); //todo: cleanup constructors and parameter order

        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warn("Could not close config file. Loading default settings.", ioe);
        }

        return parameters;
    }

    private BackendParameters readBackendParameters(String propertiesFile)
    {
        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            input = this.openFileStream(propertiesFile);
            prop.load(input);
        } catch (IOException ioe)
        {
            this.log.warn("Could not open config file. Loading default settings.", ioe);
            return new BackendParameters();
        }

        boolean debugWithoutMAAS = Boolean.parseBoolean(prop.getProperty(MAAS_DEBUG_KEY));
        boolean debugWithoutBackBone = Boolean.parseBoolean(prop.getProperty(BACKBONE_DEBUG_KEY));

        String restURLMAAS = prop.getProperty(MAAS_URL_KEY);
        String restURLBackBone = prop.getProperty(BACKBONE_URL_KEY);

        this.log.info("Backend config loaded.");

        BackendParameters backendParameters = new BackendParameters(this.readParameters(propertiesFile), debugWithoutMAAS, debugWithoutBackBone, restURLMAAS, restURLBackBone);

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
            input = this.openFileStream(propertiesFile);
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

    public ParameterConfig()
    {
        this.log = LoggerFactory.getLogger(ParameterConfig.class);
    }

    @Bean
    Parameters parameters()
    {
        return this.readParameters(this.DEFAULT_PROPERTIES_FILE);
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

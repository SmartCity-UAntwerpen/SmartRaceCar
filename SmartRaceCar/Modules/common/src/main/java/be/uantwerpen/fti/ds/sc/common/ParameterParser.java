package be.uantwerpen.fti.ds.sc.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Properties;

@Deprecated
public class ParameterParser
{
    // MQTT PARAMETER KEYS
    private static final String MQTT_BROKER_KEY = "mqtt.broker";
    private static final String MQTT_USERNAME_KEY = "mqtt.username";
    private static final String MQTT_PASSWORD_KEY = "mqtt.password";
    private static final String MQTT_TOPIC_KEY = "mqtt.topic";

    // RACECAR KEYS
    private static final String RACECAR_URL_KEY = "Racecar.URL";

    protected Logger log;

    private Parameters readParameters(String propertiesFile)
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

    public FileInputStream openFileStream(String file) throws IOException
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

    public ParameterParser()
    {
        this.log = LoggerFactory.getLogger(ParameterParser.class);
    }

    public Parameters parse(String file)
    {
        return this.readParameters(file);
    }
}

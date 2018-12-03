package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.LogbackWrapper;
import be.uantwerpen.fti.ds.sc.smartracecar.common.Parameters;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.Optional;
import java.util.Properties;

public class RacecarBackendV2
{
    private static final String DEFAULT_PROPERTIES_FILE = "RaceCarBackend.properties";

    private LogbackWrapper log;
    private JobDispatcher jobDispatcher;
    private MapManager mapManager;
    private VehicleManager vehicleManager;

    private FileInputStream openFileStream (String file) throws IOException
    {
        try
        {
            String decodedPath = URLDecoder.decode(file, "UTF-8");
            return new FileInputStream(decodedPath);
        }
        catch (UnsupportedEncodingException uee)
        {
            // Catch, Log and re-throw
            this.log.warning("RACECAR-BACKEND", "Could not decode file path. UnsupportedEncodingException: \"" + uee.getMessage() + "\"");
            IOException ioe = new IOException(uee.getMessage());
            ioe.setStackTrace(uee.getStackTrace());
            throw ioe;
        }
        catch (IOException ioe)
        {
            // Catch, Log and re-throw
            this.log.warning("RACECAR-BACKEND", "Could not open file. IOException: \"" + ioe.getMessage() + "\"");
            throw ioe;
        }
    }

    private Parameters readParameters(Optional<String> propertiesFile)
    {
        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            input = this.openFileStream(propertiesFile.orElse(DEFAULT_PROPERTIES_FILE));
            prop.load(input);
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not open config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
            return new BackendParameters(true, true);
        }

        String mqttBroker = "tcp://" + prop.getProperty("mqttBroker");
        String mqttUsername = prop.getProperty("mqqtUsername");
        String mqttPassword = prop.getProperty("mqttPassword");
        String restCarmanagerURL = prop.getProperty("restURLBackend");

        this.log.info("RACECAR-BACKEND", "Config loaded");

        Parameters parameters = new Parameters(mqttBroker, mqttUsername, mqttPassword, restCarmanagerURL);

        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not close config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
        }

        return parameters;
    }

    private BackendParameters readBackendParameters(Optional<String> propertiesFile)
    {
        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            input = this.openFileStream(propertiesFile.orElse(DEFAULT_PROPERTIES_FILE));
            prop.load(input);
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not open config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
            return new BackendParameters(true, true);
        }

        boolean debugWithoutBackBone = Boolean.parseBoolean(prop.getProperty("debugWithoutBackBone"));
        boolean debugWithoutMAAS = Boolean.parseBoolean(prop.getProperty("debugWithoutMAAS"));

        String restURLMAAS = prop.getProperty("restURLMAAS");
        String restURLBackBone = prop.getProperty("restURLBackBone");
        String currentMap = prop.getProperty("currentMap");
        String mapsPath = prop.getProperty("mapsPath");

        this.log.info("RACECAR-BACKEND", "Config loaded");

        BackendParameters backendParameters = new BackendParameters(this.readParameters(propertiesFile), debugWithoutMAAS, debugWithoutBackBone, restURLMAAS, restURLBackBone);

        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not close config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
        }

        return backendParameters;
    }

    private MapManagerParameters readMapManagerParameters(Optional<String> propertiesFile)
    {
        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            input = this.openFileStream(propertiesFile.orElse(DEFAULT_PROPERTIES_FILE));
            prop.load(input);
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not open config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
            return new MapManagerParameters();
        }

        String currentMap = prop.getProperty("currentMap");
        String mapPath = prop.getProperty("mapsPath");

        this.log.info("RACECAR-BACKEND", "Config loaded");

        MapManagerParameters mapManagerParameters = new MapManagerParameters(this.readBackendParameters(propertiesFile), currentMap, mapPath);
        try
        {
            input.close();
        }
        catch (IOException ioe)
        {
            this.log.warning("RACECAR-BACKEND", "Could not close config file. Loading default settings. IOException: \"" + ioe.getMessage() + "\"");
        }

        return mapManagerParameters;
    }

    public RacecarBackendV2(Optional<String> configPath)
    {
		this.log = new LogbackWrapper();

        Parameters parameters = this.readParameters(configPath);
        BackendParameters backendParameters = this.readBackendParameters(configPath);
        MapManagerParameters mapManagerParameters = this.readMapManagerParameters(configPath);

        this.mapManager = new MapManager(mapManagerParameters, null);
        this.vehicleManager = new VehicleManager(backendParameters, this.mapManager);
        this.mapManager.setVehicleManager(vehicleManager);
        this.jobDispatcher = new JobDispatcher(parameters, this.mapManager, this.vehicleManager);
    }

    public static void main (String[] args)
    {
        Optional<String> configPath;

        if (args.length == 1)
        {
            configPath = Optional.of(args[0]);
        }
        else
        {
            configPath = Optional.empty();
        }

        final RacecarBackendV2 racecarBackendV2 = new RacecarBackendV2(configPath);
    }
}

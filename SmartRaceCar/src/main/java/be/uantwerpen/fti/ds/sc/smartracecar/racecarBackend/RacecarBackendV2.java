package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Log;
import be.uantwerpen.fti.ds.sc.smartracecar.common.LogbackWrapper;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URLDecoder;
import java.util.Optional;
import java.util.Properties;
import java.util.logging.Level;

public class RacecarBackendV2
{
    private static final String DEFAULT_PROPERTIES_FILE = "RaceCarBackend.properties";

    private LogbackWrapper log;
    private JobDispatcher jobDispatcher;
    private MapManager mapManager;
    private VehicleManager vehicleManager;

    private VehicleManagerParameters readVehicleManagerParameters(Optional<String> propertiesFile)
    {
        Properties prop = new Properties();
        InputStream input = null;

        try
        {
            String path;

            path = propertiesFile.orElse(DEFAULT_PROPERTIES_FILE);

            String decodedPath = URLDecoder.decode(path, "UTF-8");
            //decodedPath = decodedPath.replace("RacecarBackend.jar", "");
            //input = new FileInputStream(decodedPath + "/racecarbackend.properties");
            input = new FileInputStream(decodedPath);
            prop.load(input);

            boolean debugWithoutBackBone = Boolean.parseBoolean(prop.getProperty("debugWithoutBackBone"));
            boolean debugWithoutMAAS = Boolean.parseBoolean(prop.getProperty("debugWithoutMAAS"));
            String mqttBroker = "tcp://" + prop.getProperty("mqttBroker");
            String mqqtUsername = prop.getProperty("mqqtUsername");
            String mqttPassword = prop.getProperty("mqttPassword");
            String restURLMAAS = prop.getProperty("restURLMAAS");
            String restURLBackBone = prop.getProperty("restURLBackBone");
            String restURLBackend = prop.getProperty("restURLBackend");
            String currentMap = prop.getProperty("currentMap");
            String mapsPath = prop.getProperty("mapsPath");
            this.log.info("RACECAR_BACKEND", "Config loaded");
        }
        catch (IOException ex)
        {
            log = new Log(this.getClass(), Level.CONFIG);
            Log.logWarning("RACECAR_BACKEND", "Could not read config file. Loading default settings. " + ex);
        }
        finally
        {
            if (input != null)
            {
                try
                {
                    input.close();
                } catch (IOException e)
                {
                    Log.logWarning("RACECAR_BACKEND", "Could not read config file. Loading default settings. " + e);
                }
            }
        }
    }
    }

    public RacecarBackendV2()
    {
        this.log = new LogbackWrapper();
        this.mapManager = new MapManager();
        this.vehicleManager = new VehicleManager(, this.mapManager);
        this.jobDispatcher = new JobDispatcher(, this.mapManager, this.vehicleManager);
    }
}

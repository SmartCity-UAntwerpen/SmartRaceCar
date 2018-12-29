package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.stereotype.Service;

@Service
public class ConfigurationService
{
    @Value("${Racecar.config_file}")
    private String DEFAULT_PROPERTIES_FILE;

    private Logger log;

    public ConfigurationService()
    {
        this.log = LoggerFactory.getLogger(ConfigurationService.class);
    }

    @Bean
    @Qualifier("costCache")
    Configuration costCacheConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.ROS);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("heartbeatChecker")
    Configuration heartbeatCheckerConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);
        configuration.add(AspectType.RACECAR);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("jobDispatcher")
    Configuration jobDispatcherConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("jobTracker")
    Configuration jobTrackerConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);
        configuration.add(AspectType.BACKBONE);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("mapManager")
    Configuration mapManagerConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);
        configuration.add(AspectType.MAP_MANAGER);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("navigationManager")
    Configuration navigationManagerConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }

    @Bean
    @Qualifier("vehicleManager")
    Configuration vehicleManagerConfiguration()
    {
        Configuration configuration = new Configuration();
        configuration.add(AspectType.MQTT);

        return configuration.load(DEFAULT_PROPERTIES_FILE);
    }
}

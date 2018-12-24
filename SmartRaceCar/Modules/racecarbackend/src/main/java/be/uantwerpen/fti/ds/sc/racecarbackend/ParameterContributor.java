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
public class ParameterContributor
{
    @Value("${Racecar.config_file}")
    private String DEFAULT_PROPERTIES_FILE;

    private Logger log;

    public ParameterContributor ()
    {
        this.log = LoggerFactory.getLogger(ParameterContributor.class);
    }

    @Bean
    @Qualifier("vehicleManager")
    Configuration vehicleManagerParameters()
    {
       Configuration configuration = new Configuration();
       configuration.add(AspectType.MQTT);

       return configuration.load(DEFAULT_PROPERTIES_FILE);
    }
}

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class ParameterConfig
{
    @Bean
    Parameters parameters()
    {
        return new Parameters();
    }

    @Bean
    @Qualifier("backend")
    BackendParameters backendParameters()
    {
        return new BackendParameters();
    }

    @Bean
    @Qualifier("mapmanager")
    MapManagerParameters mapManagerParameters()
    {
        return new MapManagerParameters();
    }
}

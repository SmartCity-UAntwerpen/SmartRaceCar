package be.uantwerpen.fti.ds.smartracecar.manager;

import org.glassfish.jersey.server.ResourceConfig;

public class JerseyConfiguration extends ResourceConfig {

    public JerseyConfiguration() {
        packages("be.uantwerpen.fti.ds.smartracecar.manager");
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.manager;

import org.glassfish.jersey.server.ResourceConfig;

class JerseyConfiguration extends ResourceConfig {

    JerseyConfiguration() {
        packages("be.uantwerpen.fti.ds.sc.smartracecar.manager");
    }
}

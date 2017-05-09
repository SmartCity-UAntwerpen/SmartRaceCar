package be.uantwerpen.fti.ds.smartracecar.manager;

import be.uantwerpen.fti.ds.smartracecar.common.*;

import java.util.logging.Level;

public class Manager implements MQTTListener{

    private Log log;
    private Level level = Level.CONFIG; //Debug level
    private final String mqttBroker = "tcp://broker.hivemq.com:1883";
    private final String mqqtUsername = "username";
    private final String mqttPassword = "password";
    private final String mapFolder = "maps";

    private RESTUtils restUtils;
    private MQTTUtils mqttUtils;
    private JSONUtils jsonUtils;

    public Manager() throws InterruptedException {
        log = new Log(this.getClass(),level);
        jsonUtils = new JSONUtils();
        mqttUtils = new MQTTUtils(mqttBroker,mqqtUsername,mqttPassword,this);
        mqttUtils.subscribeToTopic("racecar/#");
        while(true){

            log.logInfo("MANAGER","test");
            Thread.sleep(2000);
        }
    }

    public static void main(String[] args) throws InterruptedException {

        final Manager manager = new Manager();
//    }

    }

    @Override
    public void parseMQTT(String topic, String message) {

    }
}

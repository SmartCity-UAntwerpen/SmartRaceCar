package be.uantwerpen.fti.ds.sc.smartracecar.common;

//All MQTT functionality
public interface MQTTListener {
    void parseMQTT(String topic, String message);
}

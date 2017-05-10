package be.uantwerpen.fti.ds.smartracecar.common;

import org.eclipse.paho.client.mqttv3.*;

public class MQTTUtils implements MqttCallback{

    private MqttClient client;
    private MQTTListener listener;

    public MQTTUtils(String brokerURL, String username, String password, MQTTListener listener){
        MqttConnectOptions options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        //options.setUserName(username);
        //options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL,client.generateClientId());
            client.setCallback(this);
            client.connect(options);
            Log.logConfig("MQTT","Connected to '" + brokerURL + "'.");
        } catch (MqttException e) {
            Log.logSevere("MQTT","Could not connect to '" + brokerURL + "'." + e);
        }
    }

    public MQTTUtils(long ID, String brokerURL, String username, String password, MQTTListener listener){
        MqttConnectOptions options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        //options.setUserName(username);
        //options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL,String.valueOf(ID));
            client.setCallback(this);
            client.connect(options);
            Log.logConfig("MQTT","Connected to '" + brokerURL + "'.");
        } catch (MqttException e) {
            Log.logSevere("MQTT","Could not connect to '" + brokerURL + "'." + e);
        }
    }

    @Override
    public void connectionLost(Throwable t) {
        Log.logSevere("MQTT","Connection lost.");
        t.printStackTrace();
    }

    @Override
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        String message = new String(mqttMessage.getPayload());
        Log.logConfig("MQTT","message arrived. Topic:" + topic + " | Message:" + message);
        listener.parseMQTT(topic,message);
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        Log.logConfig("MQTT","Publish complete.");
    }

    public void subscribeToTopic(String topic){
        try {
            int subQoS = 0;
            client.subscribe(topic, subQoS);
            Log.logConfig("MQTT","Subscribed to topic '" + topic + "'.");
        } catch (Exception e) {
            Log.logSevere("MQTT","Could not subscribe to topic '" + topic + "'." + e);
        }
    }

    public void publishMessage(String topic, String message){
        MqttMessage mqttMessage = new MqttMessage(message.getBytes());
        mqttMessage.setRetained(false);
        Log.logConfig("MQTT","Publishing. Topic:" + topic + " | Message:" + message);
        MqttTopic mqttTopic = client.getTopic(topic);
        MqttDeliveryToken token;
        try {
            token = mqttTopic.publish(mqttMessage);
            token.waitForCompletion();
        } catch (Exception e) {
            Log.logSevere("MQTT","Could not Publish." + e);
        }
    }
}

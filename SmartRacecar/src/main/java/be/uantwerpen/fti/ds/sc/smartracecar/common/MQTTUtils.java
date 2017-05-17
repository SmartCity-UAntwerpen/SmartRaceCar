package be.uantwerpen.fti.ds.sc.smartracecar.common;

import org.eclipse.paho.client.mqttv3.*;

public class MQTTUtils implements MqttCallback{

    private MqttClient client;
    private MQTTListener listener;

    public MQTTUtils(String brokerURL, String username, String password, MQTTListener listener){
        MqttConnectOptions options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        options.setUserName(username);
        options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL, MqttClient.generateClientId());
            client.setCallback(this);
            client.connectWithResult(options);
            Log.logConfig("MQTT","Connected to '" + brokerURL + "'.");
        } catch (MqttException e) {
            Log.logSevere("MQTT","Could not connect to '" + brokerURL + "'." + e);
            System.exit(0);
        }
    }

    public MQTTUtils(long ID, String brokerURL, String username, String password, MQTTListener listener){
        MqttConnectOptions options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        options.setUserName(username);
        options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL,String.valueOf(ID));
            client.setCallback(this);
            client.connectWithResult(options);
            Log.logConfig("MQTT","Connected to '" + brokerURL + "'.");
        } catch (MqttException e) {
            Log.logSevere("MQTT","Could not connect to '" + brokerURL + "'." + e);
            System.exit(0);
        }
    }

    @Override
    public void connectionLost(Throwable t) {
        Log.logSevere("MQTT","Connection lost.");
        t.printStackTrace();
        System.exit(0);
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
            int subQoS = 2;
            client.subscribe(topic, subQoS);
            Log.logConfig("MQTT","Subscribed to topic '" + topic + "'.");
        } catch (Exception e) {
            Log.logSevere("MQTT","Could not subscribe to topic '" + topic + "'." + e);
        }
    }

    public void publishMessage(String topic, String message){
        MqttMessage mqttMessage = new MqttMessage(message.getBytes());
        mqttMessage.setRetained(false);
        mqttMessage.setQos(2);
        Log.logConfig("MQTT","Publishing. Topic:" + topic + " | Message:" + message);
        MqttTopic mqttTopic = client.getTopic(topic);

        try {
            mqttTopic.publish(mqttMessage);
        } catch (Exception e) {
            Log.logSevere("MQTT","Could not Publish." + e);
        }
    }
}

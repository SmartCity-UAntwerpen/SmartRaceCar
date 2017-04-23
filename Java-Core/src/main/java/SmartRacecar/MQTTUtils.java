package SmartRacecar;

import org.eclipse.paho.client.mqttv3.*;

public class MQTTUtils implements MqttCallback {

    MqttClient client;
    MqttConnectOptions options;

    static final String BROKER_URL = "tcp://broker.hivemq.com:1883";
    static final String CLIENT_ID = "ID";
    static final String VEHICLE_ID = "smartracecar";
    static final String USERNAME = "username";
    static final String PASSWORD = "password";


    public MQTTUtils(){
        String clientID = CLIENT_ID;
        options = new MqttConnectOptions();

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        //options.setUserName(USERNAME);
        //options.setPassword(PASSWORD.toCharArray());

        try {
            client = new MqttClient(BROKER_URL, clientID);
            client.setCallback(this);
            client.connect(options);
            System.out.println("[MQTT] [DEBUG] Connected to " + BROKER_URL);
        } catch (MqttException e) {
            System.err.println("[MQTT] [ERROR] Could not connect to " + BROKER_URL + "." + e);
        }

        String topic = VEHICLE_ID + "/" + CLIENT_ID;
        subscribeToTopic(topic);
    }

    @Override
    public void connectionLost(Throwable t) {
        System.err.println("[MQTT] [ERROR] Connection lost.");
    }

    @Override
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        System.out.println("[MQTT] [DEBUG] message arrived. Topic:" + topic + " | Message:" + new String(mqttMessage.getPayload()));
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
            System.out.println("[MQTT] [DEBUG] Publish complete.");
    }

    public void subscribeToTopic(String topic){
        try {
            int subQoS = 0;
            client.subscribe(topic, subQoS);
            System.out.println("[MQTT] [DEBUG] Subscribed to topic:" + topic);
        } catch (Exception e) {
            System.err.println("[MQTT] [ERROR] Could not subscribe to topic:" + topic + "." + e);
        }
    }

    public void publishMessage(String topic,String message){
        MqttMessage mqttMessage = new MqttMessage(message.getBytes());
        int pubQoS = 0;
        mqttMessage.setQos(pubQoS);
        mqttMessage.setRetained(false);

        System.out.println("[MQTT] [DEBUG] Publishing. Topic:" + topic + " | QoS " + pubQoS + " | Message:" + message.toString());
        MqttTopic mqttTopic = client.getTopic(topic);
        MqttDeliveryToken token = null;
        try {
            token = mqttTopic.publish(mqttMessage);
            token.waitForCompletion();
        } catch (Exception e) {
            System.err.println("[MQTT] [ERROR] Could not Publish." + e);
        }
    }
}

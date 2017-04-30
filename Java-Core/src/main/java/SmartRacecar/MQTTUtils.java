package SmartRacecar;

import org.eclipse.paho.client.mqttv3.*;

public class MQTTUtils implements MqttCallback {

    MqttClient client;
    MqttConnectOptions options;
    eventListener listener;

    public MQTTUtils(int ID,String brokerURL, String username, String password,eventListener listener){
        String clientID = String.valueOf(ID);
        options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        //options.setUserName(username);
        //options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL,client.generateClientId());
            client.setCallback(this);
            client.connect(options);
            System.out.println("[MQTT] [DEBUG] Connected to " + brokerURL);
        } catch (MqttException e) {
            System.err.println("[MQTT] [ERROR] Could not connect to " + brokerURL + "." + e);
        }

        String topic = clientID;
        subscribeToTopic(topic);
    }

    @Override
    public void connectionLost(Throwable t) {
        t.printStackTrace();
        System.err.println("[MQTT] [ERROR] Connection lost.");
    }

    @Override
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        System.out.println("[MQTT] [DEBUG] message arrived. Topic:" + topic + " | Message:" + new String(mqttMessage.getPayload()));
        //TODO Proper MQTT message handling
        jobRequest();
    }

    public void jobRequest(){
        int[] waypointsTest = {1,2,3,4};
        listener.jobRequest(waypointsTest);
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

    public void closeMQTT(){
        try {
            client.disconnect();
        } catch (MqttException e) {
            System.err.println("[MQTT] [ERROR] MqttException:  " + e);
        }
    }
}

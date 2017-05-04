package SmartRacecar;

import org.eclipse.paho.client.mqttv3.*;

class MQTTUtils implements MqttCallback {

    private MqttClient client;
    private MqttConnectOptions options;
    private CoreEvents listener;

    MQTTUtils(int ID, String brokerURL, String username, String password, CoreEvents listener){
        String clientID = String.valueOf(ID);
        options = new MqttConnectOptions();
        this.listener = listener;

        options.setCleanSession(true);
        options.setKeepAliveInterval(30);
        //options.setUserName(username);
        //options.setPassword(password.toCharArray());

        try {
            client = new MqttClient(brokerURL,clientID);
            client.setCallback(this);
            client.connect(options);
            Core.logConfig("MQTT","Connected to '" + brokerURL + "'.");
        } catch (MqttException e) {
            Core.logSevere("MQTT","Could not connect to '" + brokerURL + "'." + e);
        }

        subscribeToTopic(clientID);
    }

    @Override
    public void connectionLost(Throwable t) {
        Core.logSevere("MQTT","Connection lost.");
        t.printStackTrace();
    }

    @Override
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        Core.logConfig("MQTT","message arrived. Topic:" + topic + " | Message:" + new String(mqttMessage.getPayload()));
        //TODO Proper MQTT message handling
        jobRequest();
    }

    private void jobRequest(){
        int[] wayPoints = {4};
        listener.jobRequest(wayPoints);
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        Core.logConfig("MQTT","Publish complete.");
    }

    private void subscribeToTopic(String topic){
        try {
            int subQoS = 0;
            client.subscribe(topic, subQoS);
            Core.logConfig("MQTT","Subscribed to topic:" + topic + ".");
        } catch (Exception e) {
            Core.logSevere("MQTT","Could not subscribe to topic:" + topic + "." + e);
        }
    }

    void publishMessage(String topic,String message){
        MqttMessage mqttMessage = new MqttMessage(message.getBytes());
        int pubQoS = 0;
        mqttMessage.setQos(pubQoS);
        mqttMessage.setRetained(false);
        Core.logConfig("MQTT","Publishing. Topic:" + topic + " | QoS " + pubQoS + " | Message:" + message);
        MqttTopic mqttTopic = client.getTopic(topic);
        MqttDeliveryToken token;
        try {
            token = mqttTopic.publish(mqttMessage);
            token.waitForCompletion();
        } catch (Exception e) {
            Core.logSevere("MQTT","Could not Publish." + e);
        }
    }

    void closeMQTT(){
        try {
            client.disconnect();
        } catch (MqttException e) {
            Core.logSevere("MQTT","Could not close MQTT connection. MqttException:  " + e);
        }
    }
}

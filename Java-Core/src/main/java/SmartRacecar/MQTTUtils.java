package SmartRacecar;

import org.eclipse.paho.client.mqttv3.*;

//All MQTT functionality
class MQTTUtils implements MqttCallback {

    private MqttClient client;
    private CoreEvents listener;

    MQTTUtils(int ID, String brokerURL, String username, String password, CoreEvents listener){
        String clientID = String.valueOf(ID);
        MqttConnectOptions options = new MqttConnectOptions();
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

        subscribeToTopic("racecar/" + clientID +"/#");
    }

    @Override
    public void connectionLost(Throwable t) {
        Core.logSevere("MQTT","Connection lost.");
        t.printStackTrace();
    }

    @Override
    //Parses the topic to make the right method call.
    public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
        String message = new String(mqttMessage.getPayload());
        Core.logConfig("MQTT","message arrived. Topic:" + topic + " | Message:" + message);
        if(topic.equals("racecar/" + client.getClientId() + "/job")){
            String[] waypointStringValues = message.split(" ");
            int[] waypointValues = new int[waypointStringValues.length];
            for (int index = 0; index < waypointStringValues.length; index++) {
                waypointValues[index] = Integer.parseInt(waypointStringValues[index]);
            }
            listener.jobRequest(waypointValues);
        }
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
        Core.logConfig("MQTT","Publish complete.");
    }

    private void subscribeToTopic(String topic){
        try {
            int subQoS = 0;
            client.subscribe(topic, subQoS);
            Core.logConfig("MQTT","Subscribed to topic '" + topic + "'.");
        } catch (Exception e) {
            Core.logSevere("MQTT","Could not subscribe to topic '" + topic + "'." + e);
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
}

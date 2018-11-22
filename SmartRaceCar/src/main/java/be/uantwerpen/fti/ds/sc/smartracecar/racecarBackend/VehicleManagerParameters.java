package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Parameters;

public class VehicleManagerParameters extends Parameters
{
    private boolean disableMaaS;
    private boolean disableBackbone;

    public VehicleManagerParameters(boolean disableMaaS, boolean disableBackbone)
    {
        super();
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
    }

    public VehicleManagerParameters(String mqttBroker, String mqttUsername, String mqttPassword, String mqttTopic, String restUrl, boolean disableMaaS, boolean disableBackbone)
    {
        super(mqttBroker, mqttUsername, mqttPassword, mqttTopic, restUrl);
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
    }

    public boolean isMaaSDisabled()
    {
        return this.disableMaaS;
    }

    public boolean getBackboneDisabled()
    {
        return this.disableBackbone;
    }

    public void setMaasDisabled(boolean disableMaaS)
    {
        this.disableMaaS = disableMaaS;
    }

    public void setBackboneDisabled(boolean disableBackbone)
    {
        this.disableBackbone = disableBackbone;
    }
}

package be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Parameters;

public class VehicleManagerParameters extends Parameters
{
    private boolean disableMaaS;
    private boolean disableBackbone;
    private String MaaSRESTUrl;
    private String backboneRESTUrl;

    public VehicleManagerParameters(boolean disableMaaS, boolean disableBackbone)
    {
        super();
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
        this.MaaSRESTUrl = "http://smartcity.ddns.net:8090";
        this.backboneRESTUrl = "http://smartcity.ddns.net:8090";
    }

    public VehicleManagerParameters(boolean disableMaaS, boolean disableBackbone, String MaasRESTUrl, String backboneRESTUrl)
    {
        super();
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
        this.MaaSRESTUrl = MaasRESTUrl;
        this.backboneRESTUrl = backboneRESTUrl;
    }

    public VehicleManagerParameters(String mqttBroker, String mqttUsername, String mqttPassword, String mqttTopic, String restUrl, boolean disableMaaS, boolean disableBackbone)
    {
        super(mqttBroker, mqttUsername, mqttPassword, mqttTopic, restUrl);
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
        this.MaaSRESTUrl = "http://smartcity.ddns.net:8090";
        this.backboneRESTUrl = "http://smartcity.ddns.net:10000";
    }

    public VehicleManagerParameters(String mqttBroker, String mqttUsername, String mqttPassword, String mqttTopic, String restUrl, boolean disableMaaS, boolean disableBackbone, String MaasRESTUrl, String backboneRESTUrl)
    {
        super(mqttBroker, mqttUsername, mqttPassword, mqttTopic, restUrl);
        this.disableMaaS = disableMaaS;
        this.disableBackbone = disableBackbone;
        this.MaaSRESTUrl = MaasRESTUrl;
        this.backboneRESTUrl = backboneRESTUrl;
    }

    public boolean isMaaSDisabled()
    {
        return this.disableMaaS;
    }

    public boolean getBackboneDisabled()
    {
        return this.disableBackbone;
    }

    public String getMaaSRESTUrl()
    {
        return this.MaaSRESTUrl;
    }

    public String getBackboneRESTUrl()
    {
        return this.backboneRESTUrl;
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

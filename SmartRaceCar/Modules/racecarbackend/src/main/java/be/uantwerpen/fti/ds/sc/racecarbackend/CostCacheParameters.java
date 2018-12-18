package be.uantwerpen.fti.ds.sc.racecarbackend;

public class CostCacheParameters extends BackendParameters
{
    private static final boolean DEFAULT_ROS_DEBUG = true;
    private static final String DEFAULT_ROS_SERVER_URL = "http://smartcity.ddns.net:8084";

    private boolean isRosServerDisabled;
    private String rosServerURL;

    public CostCacheParameters()
    {
        this(new BackendParameters(), DEFAULT_ROS_DEBUG, DEFAULT_ROS_SERVER_URL);
    }

    public CostCacheParameters(BackendParameters backendParameters, boolean isRosServerDisabled, String rosServerURL)
    {
        super(backendParameters);
        this.isRosServerDisabled = isRosServerDisabled;
        this.rosServerURL = rosServerURL;
    }

    public boolean isROSServerDisabled()
    {
        return this.isRosServerDisabled;
    }

    public String getROSServerURL()
    {
        return this.rosServerURL;
    }
}

package be.uantwerpen.fti.ds.sc.simkernel;

public class SimkernelParameters
{
    private boolean isRosServerDisabled;
    private String rosServerURL;

    public SimkernelParameters(boolean isRosServerDisabled, String rosServerURL)
    {
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

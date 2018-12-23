package be.uantwerpen.fti.ds.sc.simkernel;

import be.uantwerpen.fti.ds.sc.common.Parameters;

public class SimkernelParameters extends Parameters
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

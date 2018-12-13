package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.springframework.beans.factory.annotation.Value;

public class BackendParameters extends Parameters
{
	private boolean disableMaaS;
	private String MaaSRESTUrl;

	private boolean disableBackbone;
	private String backboneRESTUrl;

	public BackendParameters()
	{
		super();
		this.disableMaaS = true;
		this.MaaSRESTUrl = "http://smartcity.ddns.net:8090";
		this.disableBackbone = true;
		this.backboneRESTUrl = "http://smartcity.ddns.net:8090";
	}

	public BackendParameters(BackendParameters backendParameters)
	{
		super(backendParameters);
		this.disableMaaS = backendParameters.isMaaSDisabled();
		this.disableBackbone = backendParameters.isBackboneDisabled();
		this.MaaSRESTUrl = backendParameters.getMaaSRESTUrl();
		this.backboneRESTUrl = backendParameters.getBackboneRESTURL();
	}

	public BackendParameters(Parameters parameters, boolean disableMaaS, boolean disableBackbone, String MaasRESTUrl, String backboneRESTUrl)
	{
		super(parameters);
		this.disableMaaS = disableMaaS;
		this.disableBackbone = disableBackbone;
		this.MaaSRESTUrl = MaasRESTUrl;
		this.backboneRESTUrl = backboneRESTUrl;
	}

	public boolean isMaaSDisabled()
	{
		return this.disableMaaS;
	}

	public boolean isBackboneDisabled()
	{
		return this.disableBackbone;
	}

	public String getMaaSRESTUrl()
	{
		return this.MaaSRESTUrl;
	}

	public String getBackboneRESTURL()
	{
		return this.backboneRESTUrl;
	}
}

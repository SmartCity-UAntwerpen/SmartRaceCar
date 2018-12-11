package be.uantwerpen.fti.ds.sc.smartracecar;

import org.springframework.beans.factory.annotation.Value;

public class BackendParameters extends Parameters
{
	@Value("${MaaS.debug}")
	private boolean disableMaaS;

	@Value("${MaaS.URL}")
	private String MaaSRESTUrl;

	@Value("${Backbone.debug}")
	private boolean disableBackbone;

	@Value("${Backbone.URL}")
	private String backboneRESTUrl;

	public BackendParameters()
	{
		super();
		/*
		this.disableMaaS = true;
		this.disableBackbone = true;
		this.MaaSRESTUrl = "http://smartcity.ddns.net:8090";
		this.backboneRESTUrl = "http://smartcity.ddns.net:8090";
		*/
	}

	@Deprecated
	public BackendParameters(BackendParameters backendParameters)
	{
		super(backendParameters);
		this.disableMaaS = backendParameters.isMaaSDisabled();
		this.disableBackbone = backendParameters.isBackboneDisabled();
		this.MaaSRESTUrl = backendParameters.getMaaSRESTUrl();
		this.backboneRESTUrl = backendParameters.getBackboneRESTURL();
	}

	@Deprecated
	public BackendParameters(boolean disableMaaS, boolean disableBackbone)
	{
		super();
		this.disableMaaS = disableMaaS;
		this.disableBackbone = disableBackbone;
		this.MaaSRESTUrl = "http://smartcity.ddns.net:8090";
		this.backboneRESTUrl = "http://smartcity.ddns.net:8090";
	}

	@Deprecated
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

	@Deprecated
	public void setMaasDisabled(boolean disableMaaS)
	{
		this.disableMaaS = disableMaaS;
	}

	@Deprecated
	public void setBackboneDisabled(boolean disableBackbone)
	{
		this.disableBackbone = disableBackbone;
	}
}

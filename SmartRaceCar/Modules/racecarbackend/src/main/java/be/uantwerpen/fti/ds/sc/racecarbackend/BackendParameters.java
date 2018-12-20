package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.Parameters;
import org.springframework.beans.factory.annotation.Value;

public class BackendParameters extends Parameters
{
	private boolean disableBackbone;
	private String backboneRESTUrl;

	public BackendParameters()
	{
		super();
		this.disableBackbone = true;
		this.backboneRESTUrl = "http://smartcity.ddns.net:8090";
	}

	public BackendParameters(BackendParameters backendParameters)
	{
		super(backendParameters);
		this.disableBackbone = backendParameters.isBackboneDisabled();
		this.backboneRESTUrl = backendParameters.getBackboneRESTURL();
	}

	public BackendParameters(Parameters parameters, boolean disableBackbone, String String backboneRESTUrl)
	{
		super(parameters);
		this.disableBackbone = disableBackbone;
		this.backboneRESTUrl = backboneRESTUrl;
	}

	public boolean isBackboneDisabled()
	{
		return this.disableBackbone;
	}

	public String getBackboneRESTURL()
	{
		return this.backboneRESTUrl;
	}
}

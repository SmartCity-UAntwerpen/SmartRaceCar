package be.uantwerpen.fti.ds.sc.common.configuration;

public class BackboneAspect
{
	private static final String PREFIX = "Backbone";

	private static final String BACKBONE_DEBUG_MODE_KEY = PREFIX + ".debug";
	private static final String BACKBONE_SERVER_URL_KEY = PREFIX + ".url";

	private static final String DEFAULT_BACKBONE_DEBUG_MODE = "true";
	private static final String DEFAULT_BACKBONE_SERVER_URL = "http://smartcity.ddns.net:10000";

	private boolean backboneDebugMode;
	private String backboneServerUrl;
}

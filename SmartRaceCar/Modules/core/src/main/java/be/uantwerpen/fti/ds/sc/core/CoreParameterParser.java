package be.uantwerpen.fti.ds.sc.core;

import be.uantwerpen.fti.ds.sc.common.ParameterParser;
import be.uantwerpen.fti.ds.sc.common.Parameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class CoreParameterParser extends ParameterParser
{
	private static final String DEBUG = "debugWithoutRosKernel";

	private CoreParameters readParameters(String propertiesFile)
	{
		Parameters parameters = super.parse(propertiesFile);

		Properties prop = new Properties();
		InputStream input = null;

		try
		{
			input = this.openFileStream(propertiesFile);
			prop.load(input);
		}
		catch (IOException ioe)
		{
			this.log.warn("Could not open config file. Loading default settings.", ioe);
			return new CoreParameters();
		}

		boolean debug = Boolean.parseBoolean(prop.getProperty(DEBUG));

		this.log.info("Standard config loaded.");

		CoreParameters coreParameters = new CoreParameters(parameters, debug); //todo: cleanup constructors and parameter order

		try
		{
			input.close();
		}
		catch (IOException ioe)
		{
			this.log.warn("Could not close config file. Loading default settings.", ioe);
		}

		return coreParameters;
	}

	public CoreParameters parse(String file)
	{
		return this.readParameters(file);
	}
}

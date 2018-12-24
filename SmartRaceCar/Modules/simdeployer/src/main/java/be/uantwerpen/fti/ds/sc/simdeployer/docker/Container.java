package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.Messages;
import be.uantwerpen.fti.ds.sc.simdeployer.SimDeployerParameters;
import be.uantwerpen.fti.ds.sc.simdeployer.VirtualMachine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.naming.InvalidNameException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Container implements VirtualMachine, MQTTListener
{
	private static final String DOCKER_COMMAND = "docker";
	private static final String DOCKER_RUN = "run";
	private static final String DOCKER_REMOVE = "rm";

	private static final String DOCKER_NAME_OPTION = "--name";

	private static final String DOCKER_CONTAINER_NAME_REGEX = "[a-zA-Z0-9][a-zA-Z0-9_.-]+";

	private Logger log;
	private long simulationId;
	private String imageName;
	private String containerName;
	private Process simulationProcess;
	private MQTTUtils mqttUtils;

	public Container (SimDeployerParameters parameters, long simulationId, String imageName, String containerName) throws InvalidNameException
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.log.info("Created Docker Container " + imageName + ", with Simulation ID " + simulationId);
		this.simulationId = simulationId;
		this.imageName = imageName;
		this.mqttUtils = new MQTTUtils(parameters.getMqttBroker(), parameters.getMqttUserName(), parameters.getMqttPassword(), this);

		Pattern containerNamePattern = Pattern.compile(DOCKER_CONTAINER_NAME_REGEX);
		Matcher containerNameMatcher = containerNamePattern.matcher(containerName);

		if (!containerNameMatcher.matches())
		{
			throw new InvalidNameException("Docker container got invalid name \"" + containerName + "\", ");
		}
		else
		{
			this.containerName = containerName;
		}
	}

	@Override
	public void run(List<String> args) throws IOException
	{
		List<String> commandLine = new ArrayList<>();
		commandLine.add(DOCKER_COMMAND);
		commandLine.add(DOCKER_RUN);
		commandLine.add(DOCKER_NAME_OPTION);
		commandLine.add(this.containerName);
		commandLine.add(this.imageName);
		commandLine.addAll(args);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

		String logFilePath = "./" + this.containerName + ".log";

		// Make sure the log file exists, otherwise, the process builder will crash
		File logFile = new File(logFilePath);
		logFile.getParentFile().mkdirs();
		logFile.createNewFile();

		processBuilder = processBuilder.redirectOutput(logFile);
		processBuilder = processBuilder.redirectError(logFile);

		this.log.info("Running Docker Container " + this.imageName + ", with Simulation ID " + this.simulationId);

		try
		{
			this.simulationProcess = processBuilder.start();
		}
		catch (IOException ioe)
		{
			this.log.error("Failed to start Docker process.", ioe);
			throw ioe;
		}
	}

	@Override
	public int stop() throws IOException, InterruptedException
	{
		this.log.info("Stopping Docker Container " + this.imageName + ", with Simulation ID " + this.simulationId);

		this.mqttUtils.publish("/racecar/simdeployer/" + Messages.SIMDEPLOYER.KILL +  "/" + this.simulationId, Messages.SIMDEPLOYER.KILL);

		int returnValue = this.simulationProcess.waitFor();

		List<String> commandLine = new ArrayList<>();
		commandLine.add(DOCKER_COMMAND);
		commandLine.add(DOCKER_REMOVE);
		commandLine.add(this.containerName);

		this.log.info("Removing Docker Container " + this.imageName + ", with Simulation ID " + this.simulationId);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

		try
		{
			Process process = processBuilder.start();
			process.waitFor();
			return returnValue;
		}
		catch (IOException | InterruptedException ie)
		{
			this.log.error("Failed to remove Docker process.", ie);
			throw ie;
		}
	}

	/**
	 * We don't listen for anything but we do have to provide this method to prevent problems with MQTT.
	 * @param topic   received MQTT topic
	 * @param message received MQTT message string
	 */
	@Override
	public void parseMQTT(String topic, String message)
	{
	}
}

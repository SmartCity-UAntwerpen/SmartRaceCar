package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.Messages;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.DockerAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import be.uantwerpen.fti.ds.sc.simdeployer.SimDeployerParameters;
import be.uantwerpen.fti.ds.sc.simdeployer.VirtualMachine;
import org.eclipse.paho.client.mqttv3.MqttException;
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
	private static final String DOCKER_STOP = "stop";
	private static final String DOCKER_REMOVE = "rm";

	private static final String DOCKER_NAME_OPTION = "--name";

	private static final String DOCKER_CONTAINER_NAME_REGEX = "[a-zA-Z0-9][a-zA-Z0-9_.-]+";

	private Logger log;
	private long simulationId;
	private String imageName;
	private String containerName;
	private Process simulationProcess;
	private MQTTUtils mqttUtils;

	public Container (Configuration configuration, long simulationId, String imageName, String containerName) throws InvalidNameException
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.log.info("Created Docker Container " + imageName + ", with Simulation ID " + simulationId);
		this.simulationId = simulationId;
		this.imageName = imageName;

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.mqttUtils = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MQTTUtils for Docker Container.", me);
		}

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

		try
		{
			this.mqttUtils.publish("/racecar/simdeployer/" + Messages.SIMDEPLOYER.KILL + "/" + this.simulationId, Messages.SIMDEPLOYER.KILL);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to send Kill command to simulation over MQTT. Stopping container manually.", me);
			List<String> commandLine = new ArrayList<>();
			commandLine.add(DOCKER_COMMAND);
			commandLine.add(DOCKER_STOP);
			commandLine.add(this.containerName);

			ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

			try
			{
				Process process = processBuilder.start();
				process.waitFor();
			}
			catch (InterruptedException ie)
			{
				this.log.error("Failed to stop Docker container.", ie);
				throw ie;
			}
		}

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
			this.log.error("Failed to remove Docker container.", ie);
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

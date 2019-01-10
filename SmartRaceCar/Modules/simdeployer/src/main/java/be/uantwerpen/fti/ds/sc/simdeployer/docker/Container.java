package be.uantwerpen.fti.ds.sc.simdeployer.docker;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.MqttMessages;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.DockerAspect;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import be.uantwerpen.fti.ds.sc.simdeployer.VirtualMachine;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.naming.InvalidNameException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Container implements VirtualMachine, MQTTListener
{
	private static final long TIMEOUT_LEN = 30;
	private static final TimeUnit TIMEOUT_UNIT = TimeUnit.SECONDS;

	private static final String DOCKER_CONTAINER_NAME_REGEX = "[a-zA-Z0-9][a-zA-Z0-9_.-]+";

	private Logger log;
	private Configuration configuration;
	private long simulationId;
	private String imageName;
	private String containerName;
	private Process simulationProcess;
	private MQTTUtils mqttUtils;

	private void forcedStop() throws InterruptedException, IOException
	{
		DockerCommandBuilder builder = new DockerCommandBuilder(CommandType.STOP);
		builder.addOption(new NameOption(this.containerName));

		List<String> commandLine = new ArrayList<>();
		commandLine.addAll(builder.toStringList());

		this.log.debug(Arrays.toString(commandLine.toArray())); // We print the command we're about to execute for debugging
		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

		try
		{
			Process process = processBuilder.start();
			process.waitFor();
		}
		catch (InterruptedException | IOException ie)
		{
			this.log.error("Failed to stop Docker container.", ie);
			throw ie;
		}
	}

	public Container (Configuration configuration, long simulationId, String imageName, String containerName) throws InvalidNameException
	{
		this.log = LoggerFactory.getLogger(Container.class);
		this.configuration = configuration;
		this.log.info("Creating Docker Container " + imageName + ", with Simulation ID " + simulationId);
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
		DockerAspect dockerAspect = (DockerAspect) this.configuration.get(AspectType.DOCKER);

		// Build Docker command
		DockerCommandBuilder builder = new DockerCommandBuilder(CommandType.RUN);
		builder.setImageName(this.imageName);
		builder.addOption(new NameOption(this.containerName));                                                                          // Add name optionType so we can easily track/stop the container
		builder.addOption(new MountOption(dockerAspect.isReadonly(), dockerAspect.getHostVolume(), dockerAspect.getContainerVolume())); // Add Mount optionType to make sure the container has a config file

		List<String> commandLine = new ArrayList<>();
		commandLine.addAll(builder.toStringList());  // Docker command
		commandLine.addAll(args);                    // Arguments destined for Docker container

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

		String logFilePath = "./" + this.containerName + ".log";

		// Make sure the log file exists, otherwise, the process builder will crash
		File logFile = new File(logFilePath);
		logFile.getParentFile().mkdirs();
		logFile.createNewFile();

		processBuilder = processBuilder.redirectOutput(logFile);
		processBuilder = processBuilder.redirectError(logFile);


		this.log.debug(Arrays.toString(commandLine.toArray())); // We print the command we're about to execute for debugging
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
			MqttAspect mqttAspect = (MqttAspect) this.configuration.get(AspectType.MQTT);
			this.mqttUtils.publish(mqttAspect.getTopic() + "simdeployer/" + MqttMessages.Topics.SimDeployer.KILL + "/" + this.simulationId, MqttMessages.Messages.SimDeployer.KILL);
		}
		catch (MqttException me)
		{
			this.log.error("Failed to send Kill command to simulation over MQTT. Stopping container manually.", me);

			this.forcedStop();
		}

		// We wait for 30s before we assume the container is unresponsive
		boolean normalExit = this.simulationProcess.waitFor(TIMEOUT_LEN, TIMEOUT_UNIT);

		if (!normalExit)
		{
			this.log.warn("Docker container failed to close down within the allotted time-out period (" + TIMEOUT_LEN + ' ' + TIMEOUT_UNIT + ").");

			this.forcedStop();
		}

		DockerCommandBuilder builder = new DockerCommandBuilder(CommandType.REMOVE);
		builder.addOption(new NameOption(this.containerName));

		List<String> commandLine = new ArrayList<>();
		commandLine.addAll(builder.toStringList());

		this.log.debug(Arrays.toString(commandLine.toArray())); // We print the command we're about to execute for debugging
		this.log.info("Removing Docker Container " + this.imageName + ", with Simulation ID " + this.simulationId);

		ProcessBuilder processBuilder = new ProcessBuilder(commandLine);

		try
		{
			Process process = processBuilder.start();
			process.waitFor();
			return normalExit ? this.simulationProcess.exitValue() : -1;   // If we timed out, return -1, otherwise, return the process' return value
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

package be.uantwerpen.fti.ds.sc.racecarbackend;

import be.uantwerpen.fti.ds.sc.common.MQTTListener;
import be.uantwerpen.fti.ds.sc.common.MQTTUtils;
import be.uantwerpen.fti.ds.sc.common.MessageQueueClient;
import be.uantwerpen.fti.ds.sc.common.MqttMessages;
import be.uantwerpen.fti.ds.sc.common.configuration.AspectType;
import be.uantwerpen.fti.ds.sc.common.configuration.Configuration;
import be.uantwerpen.fti.ds.sc.common.configuration.MqttAspect;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

@Service
public class JobQueue implements MQTTListener
{
	private Logger log;
	private MessageQueueClient messageQueueClient;
	private List<Job> localJobs;
	private List<Job> globalJobs;

	public JobQueue(@Qualifier("jobQueue") Configuration configuration)
	{
		this.localJobs = new LinkedList<>();
		this.globalJobs = new LinkedList<>();
		this.log = LoggerFactory.getLogger(JobQueue.class);

		this.log.info("Initializing JobQueue...");

		try
		{
			MqttAspect mqttAspect = (MqttAspect) configuration.get(AspectType.MQTT);
			this.messageQueueClient = new MQTTUtils(mqttAspect.getBroker(), mqttAspect.getUsername(), mqttAspect.getPassword(), this);
			this.messageQueueClient.subscribe(mqttAspect.getTopic() + "/" + MqttMessages.Topics.Backend.DELETE + "/#");
		}
		catch (MqttException me)
		{
			this.log.error("Failed to set up MessageQueueClient for JobQueue.", me);
		}
		catch (Exception e)
		{
			this.log.error("Failed to subscribe to delete topic for JobQueue.", e);
		}

		this.log.info("Initialized JobQueue.");
	}

	public boolean isEnqueued(long jobId, JobType type) throws NoSuchElementException
	{
		switch (type)
		{
			case GLOBAL:
				for (Job globalJob: this.globalJobs)
				{
					if (globalJob.getJobId() == jobId)
					{
						return true;
					}
				}

				return false;

			case LOCAL:
				for (Job localJob: this.localJobs)
				{
					if (localJob.getJobId() == jobId)
					{
						return true;
					}
				}

				return false;

			default:
				String errorString = "Failed to check if job " + jobId + " (Type: " + type + ") was enqueued.";
				this.log.error(errorString);
				throw new NoSuchElementException(errorString);
		}
	}

	public boolean isEmpty(JobType type) throws NoSuchElementException
	{
		switch (type)
		{
			case GLOBAL:
				return this.globalJobs.isEmpty();

			case LOCAL:
				return this.localJobs.isEmpty();

			default:
				String errorString = "Failed to check if " + type + " queue was empty.";
				this.log.error(errorString);
				throw new NoSuchElementException(errorString);
		}
	}

	public long enqueue(Job job, JobType type) throws NoSuchElementException
	{
		switch (type)
		{
			case GLOBAL:
			{
				this.globalJobs.add(job);
				int queueSize = this.globalJobs.size();
				this.log.info("Job " + job.getJobId() + " was placed in global job queue, no. " + queueSize + " in line.");
				return queueSize;
			}

			case LOCAL:
			{
				this.localJobs.add(job);
				int queueSize = this.localJobs.size();
				this.log.info("Job " + job.getJobId() + " was placed in local job queue, no. " + queueSize + " in line.");
				return queueSize;
			}

			default:
				String errorString = "Failed to check if job " + job.getJobId() + " (Type: " + type + ") was enqueued.";
				this.log.error(errorString);
				throw new NoSuchElementException(errorString);
		}
	}

	public Job dequeue(JobType type) throws NoSuchElementException
	{
		if (this.isEmpty(type))
		{
			String errorString = "Requested job from empty queue.";
			this.log.error(errorString);
			throw new NoSuchElementException(errorString);
		}

		switch (type)
		{
			case GLOBAL:
			{
				this.log.info("Dequeueing job " + this.globalJobs.get(0).getJobId() + " from local queue.");
				return this.globalJobs.remove(0);
			}

			case LOCAL:
			{
				this.log.info("Dequeueing job " + this.localJobs.get(0).getJobId() + " from local queue.");
				return this.localJobs.remove(0);
			}

			default:
				String errorString = "Failed to dequeue job from " + type + " queue.";
				this.log.error(errorString);
				throw new NoSuchElementException(errorString);
		}
	}

	@Override
	public void parseMQTT(String topic, String message)
	{
		// If a vehicle was deleted, we need to re-assign all jobs with that vehicle
		// We assign -1, this will cause the JobDispatcher to find a new vehicle for this job.
		long vehicleId = TopicUtils.getVehicleId(topic);

		// Check Local jobs
		for (Job job: this.localJobs)
		{
			if (job.getVehicleId() == vehicleId)
			{
				job.setVehicleId(-1L);
			}
		}

		// Check global jobs
		for (Job job: this.globalJobs)
		{
			if (job.getVehicleId() == vehicleId)
			{
				job.setVehicleId(-1L);
			}
		}
	}
}

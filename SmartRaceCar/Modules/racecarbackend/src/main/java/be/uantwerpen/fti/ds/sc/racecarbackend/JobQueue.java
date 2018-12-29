package be.uantwerpen.fti.ds.sc.racecarbackend;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

@Service
public class JobQueue
{
	private Logger log;
	private List<Job> localJobs;
	private List<Job> globalJobs;

	public JobQueue()
	{
		this.localJobs = new LinkedList<>();
		this.globalJobs = new LinkedList<>();
		this.log = LoggerFactory.getLogger(JobQueue.class);
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

	public boolean isEmpty(JobType type)
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

	public long enqueue(Job job, JobType type)
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

	public Job dequeue(JobType type)
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
}

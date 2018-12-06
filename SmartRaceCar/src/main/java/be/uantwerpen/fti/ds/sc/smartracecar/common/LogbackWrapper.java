package be.uantwerpen.fti.ds.sc.smartracecar.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LogbackWrapper
{
	private Logger logger;

	@Deprecated
	public LogbackWrapper()
	{
		this.logger = LoggerFactory.getLogger(this.getClass());
	}

	public LogbackWrapper(Object obj)
	{
		this.logger = LoggerFactory.getLogger(obj.getClass());
	}

	public void info(String category, String message)
	{
		this.logger.info(this.format(category, message));
	}

	public void warning(String category, String message)
	{
		this.logger.warn(this.format(category, message));
	}

	public void error(String category, String message)
	{
		this.logger.error(this.format(category, message));
	}

	private String format(String category, String message)
	{
		return "[" + category + "]\t" + message;
	}
}

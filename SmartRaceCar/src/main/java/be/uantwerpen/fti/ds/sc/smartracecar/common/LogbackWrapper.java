package be.uantwerpen.fti.ds.sc.smartracecar.common;


import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

@Service
public class LogbackWrapper
{
	private String format(String category, String message)
	{
		return "[" + category + "]\t" + message;
	}

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

	public void debug(String category, String message)
	{
		this.logger.debug(this.format(category, message));
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
}

package be.uantwerpen.fti.ds.sc.smartracecar.common;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

public class LogbackWrapper
{
    private Logger logger;

    public LogbackWrapper()
    {
        this.logger = LoggerFactory.getLogger(LogbackWrapper.class);
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

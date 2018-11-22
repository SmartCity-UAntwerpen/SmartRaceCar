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

    public void info(String message)
    {
        this.logger.info(message);
    }

    public void warning(String message)
    {
        this.logger.warn(message);
    }

    public void error(String message)
    {
        this.logger.error(message);
    }
}

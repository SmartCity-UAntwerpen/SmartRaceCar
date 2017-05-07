package be.uantwerpen.fti.ds.model;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;


//Helper class to format the logging messages to a specific format of YEAR-MONTH-DAY HOUR:MINUTES:SECONDS:MILISECONDS [LEVEL] [TYPE] Message (+ errorprinttrace)
public class LogFormatter extends Formatter {
    // Create a DateFormat to format the logger timestamp.
    private static final DateFormat df = new SimpleDateFormat("yyyy-MM-dd hh:mm:ss.SSS");

    public String format(LogRecord record) {
        StringBuilder builder = new StringBuilder(1000);
        builder.append(df.format(new Date(record.getMillis()))).append(" ");
        if(record.getLevel().equals(Level.CONFIG)){
            builder.append("[").append("DEBUG").append("] ");
        }else{
            builder.append("[").append(record.getLevel()).append("] ");
        }
        builder.append(formatMessage(record));
        builder.append("\n");
        return builder.toString();
    }

    public String getHead(Handler h) {
        return super.getHead(h);
    }

    public String getTail(Handler h) {
        return super.getTail(h);
    }


}

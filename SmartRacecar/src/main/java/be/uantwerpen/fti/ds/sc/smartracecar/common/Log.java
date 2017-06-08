package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.Level;

/**
 * Model of a Log using Java Logger.
 */
public class Log {

    private static java.util.logging.Logger logging = null; // THe Java Logger being used.

    /**
     * Model of a Log using Java Logger.
     *
     * @param t Class where the logger is being initiated.
     * @param level Level of the debugging log.
     */
    public Log(Class t, Level level){
        logging = java.util.logging.Logger.getLogger(t.getName());
        setLogger(level);
    }

    /**
     * Execute a log to console of the level INFO.
     *
     * @param category Which module is being used is defined here.
     * @param message The  log message.
     */
    public static void logInfo(String category, String message){
        logging.info("[" + category + "] " + message);
    }

    /**
     * Execute a log to console of the level WARNING.
     *
     * @param category Which module is being used is defined here.
     * @param message The  log message.
     */
    public static void logWarning(String category, String message){
        logging.warning("[" + category + "] " + message);
    }

    /**
     * Execute a log to console of the level SEVERE.
     *
     * @param category Which module is being used is defined here.
     * @param message The  log message.
     */
    public static void logSevere(String category, String message){
        logging.severe("[" + category + "] " + message);
    }

    /**
     * Execute a log to console of the level CONFIG.
     *
     * @param category Which module is being used is defined here.
     * @param message The  log message.
     */
    public static void logConfig(String category, String message){
        logging.config("[" + category + "] " + message);
    }

    /**
     * To set the logged level. This sets which level is the maximum level that would be printed out.
     *
     * @param level The level to be used.
     */
    private void setLogger(Level level){
        Handler handlerObj = new ConsoleHandler();
        handlerObj.setLevel(Level.ALL);
        LogFormatter logFormatter = new LogFormatter();
        handlerObj.setFormatter(logFormatter);
        logging.addHandler(handlerObj);
        logging.setLevel(Level.ALL);
        logging.setUseParentHandlers(false);
        logging.setLevel(level);
    }
}

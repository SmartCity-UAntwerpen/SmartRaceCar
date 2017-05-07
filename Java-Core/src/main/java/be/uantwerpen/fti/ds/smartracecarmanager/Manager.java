package be.uantwerpen.fti.ds.smartracecarmanager;

import be.uantwerpen.fti.ds.model.Log;

import java.util.logging.Level;

public class Manager {

    private Log log;

    public Manager() throws InterruptedException {
        while(true){
            log = new Log(this.getClass(),Level.INFO);
            log.logInfo("MANAGER","test");
            Thread.sleep(2000);
        }
    }

    public static void main(String[] args) throws InterruptedException {

        final Manager manager = new Manager();
//    }

    }
}

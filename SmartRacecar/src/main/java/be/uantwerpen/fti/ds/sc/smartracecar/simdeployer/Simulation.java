package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import java.util.List;

public class Simulation {


    private String location;
    private boolean running;
    private Thread thread;
    private List<String> runArguments;

    public Simulation(String location) {
        this.location = location;
        this.running = false;
        this.thread = null;
    }

    public boolean start(List<String> runArguments) {
        this.runArguments = runArguments;
        if (!running) {
            thread = new Thread(new CoreProcess());
            thread.start();
            running = true;
            return true;
        } else {
            return false;
        }
    }

    public boolean stop() {
        if (running) {
            thread.interrupt();
            return true;
        } else {
            return false;
        }
    }

    private class CoreProcess implements Runnable {
        @Override
        public void run() {
            //Create process
            ProcessBuilder processBuilder = new ProcessBuilder("java");
            processBuilder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
            processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT);
            //Add core boot arguments
            List<String> processCommands = processBuilder.command();
            processCommands.add("-jar");
            processCommands.add(location);
            processCommands.addAll(runArguments);
            processBuilder.command(processCommands);
            try {
                Process process = processBuilder.start();
            } catch (Exception e) {
                e.printStackTrace();
                running = false;
                return;
            }
        }
    }
}

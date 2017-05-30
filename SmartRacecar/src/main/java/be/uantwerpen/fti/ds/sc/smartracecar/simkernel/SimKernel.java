package be.uantwerpen.fti.ds.sc.smartracecar.simkernel;

import be.uantwerpen.fti.ds.sc.smartracecar.common.*;

import java.util.logging.Level;

class SimKernel implements TCPListener {

    private boolean debugWithoutRosServer = true; // debug parameter to stop attempts to send over sockets when ROSServer-Node is active.
    private Log log;
    private Level level = Level.CONFIG; //Debug level
    private final String restURL = "http://localhost:8080/carmanager";
    private static int serverPort = 5005;
    private static int clientPort = 5006;

    private TCPUtils tcpUtils;
    private RESTUtils restUtils;

    private boolean connected = false; // To verify socket connection to vehicle.
    private Map map;
    private WayPoint startPoint;


    SimKernel(int serverPort,int clientPort) throws InterruptedException {
        log = new Log(this.getClass(), level);
        log.logConfig("SIMKERNEL","Startup parameters: TCP Server Port:" + serverPort + " | TCP Client Port:" + clientPort);
        restUtils = new RESTUtils(restURL);
        tcpUtils = new TCPUtils(serverPort, clientPort, this);
        tcpUtils.start();
        while (!connected) {
            Thread.sleep(1);
        }
    }



    @Override
    public void parseTCP(String message) {
        if (JSONUtils.isJSONValid(message)) {
            //parses keyword to do the correct function call.
            switch (JSONUtils.getFirst(message)) {
                case "connect":
                    connectReceive();
                    break;
                case "startPoint":
                    startPoint = (WayPoint) JSONUtils.getObjectWithKeyWord(message, WayPoint.class);
                    Log.logInfo("SIMKERNEL", "Startpoint set to " + startPoint.getX() + "," + startPoint.getY() + "," + startPoint.getZ() + "," + startPoint.getW() + ".");
                    break;
                case "currentMap":
                    map = (Map) JSONUtils.getObjectWithKeyWord(message, Map.class);
                    Log.logInfo("SIMKERNEL", "Map set to '" + map.getName() + "'.");
                    break;
            }
        }
    }

    private void connectReceive() {
        tcpUtils.sendUpdate(JSONUtils.keywordToJSONString("connect"));
        connected = true;
        Log.logInfo("SIMKERNEL", "Connected to Core.");
    }


    public static void main(String[] args) throws Exception {

        if (args.length != 2) {
            System.out.println("Need 2 arguments to run. Possible arguments: tcpclientport(int) tcpserverport(int)");
            System.exit(0);
        } else if (args.length == 2) {
            if (!args[0].isEmpty()) serverPort  = Integer.parseInt(args[0]);
            if (!args[1].isEmpty()) clientPort = Integer.parseInt(args[1]);
        }
        final SimKernel simKernel = new SimKernel(serverPort,clientPort);
    }
}

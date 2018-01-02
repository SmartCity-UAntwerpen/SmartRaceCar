package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import be.uantwerpen.fti.ds.sc.smartracecar.racecarBackend.RacecarBackend;

public class RobrechtTest {

    public void test() throws Exception {
        RacecarBackend racecarBackend = new RacecarBackend(true);
        SimDeployer simDeployer = new SimDeployer();
        simDeployer.parseTCP("create 1");
        simDeployer.parseTCP("set 1 startpoint 46");
        simDeployer.parseTCP("run 1");
        Thread.sleep(1000);
        simDeployer.parseTCP("create 2");
        simDeployer.parseTCP("set 2 startpoint 47");
        simDeployer.parseTCP("run 2");
        Thread.sleep(1000);
        simDeployer.parseTCP("create 3");
        simDeployer.parseTCP("set 3 startpoint 48");
        simDeployer.parseTCP("run 3");
        //while(true);

        /*
        Assert.assertEquals(simDeployer.parseTCP(""),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("test"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("create 1"),"ACK");
        Assert.assertEquals(simDeployer.parseTCP("create 1 test"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("create 1"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("create qdf"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 startpoint 46"),"ACK");
        Assert.assertEquals(simDeployer.parseTCP("set 2 startpoint 46"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 2 startpoint 46 dqfqf"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 startpoint 45"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 name test"),"ACK");
        Assert.assertEquals(simDeployer.parseTCP("set 2 name test"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 name qdfqféàé'!éfqdfd"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 speed 10"),"ACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 speed qdfqdsf"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 2 speed 10"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("set 1 test test"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("stop 1"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("stop 2"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("stop é"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("restart 1"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("restart 2"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("restart é"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("kill 1"),"ACK");
        Assert.assertEquals(simDeployer.parseTCP("kill 2"),"NACK");
        Assert.assertEquals(simDeployer.parseTCP("kill é"),"NACK");*/
    }
}

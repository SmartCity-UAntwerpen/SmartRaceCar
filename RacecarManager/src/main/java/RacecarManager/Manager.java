package RacecarManager;

import SmartRacecar.Core;
import SmartRacecar.Map;
import SmartRacecar.Point;
import SmartRacecar.WayPoint;

public class Manager {

    Core core = new Core();
    Point p = new Point(0,0,0,0);
    Map map = new Map("title",0);
    WayPoint wp = new WayPoint(0,0,0,0,0,0);

    public Manager() throws InterruptedException {

    }
}

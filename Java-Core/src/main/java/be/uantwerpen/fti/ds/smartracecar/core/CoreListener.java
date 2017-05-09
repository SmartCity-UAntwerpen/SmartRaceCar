package be.uantwerpen.fti.ds.smartracecar.core;

import be.uantwerpen.fti.ds.smartracecar.common.Point;

/**
 * Created by Wouter Jansen on 5/9/2017.
 */
public interface CoreListener {
    void locationUpdate(Point location);
    void wayPointReached();
    void connectReceive();
}

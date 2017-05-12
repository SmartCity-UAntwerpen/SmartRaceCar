package be.uantwerpen.fti.ds.sc.smartracecar.core;

import be.uantwerpen.fti.ds.sc.smartracecar.common.Point;

interface CoreListener {
    void locationUpdate(Point location);
    void wayPointReached();
    void connectReceive();
}

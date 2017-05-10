package be.uantwerpen.fti.ds.sc.smartracecar.common;
/*
common for coordinate on a map. Uses quaternion for angles.
Y
^
|
|
0-----> X

*/

public class Point {

    private float x = 0; // x offset from 0-point
    private float y = 0; // y offset from 0-point
    private float z = 0; // quaternion z
    private float w = 0; // quaternion w

    public Point(float x, float y,float z, float w){
        this.x = x;
        this.y = y;
        this.w = w;
        this.z = z;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getW() {
        return w;
    }

    public float getZ() {
        return z;
    }
}

package SmartRacecar;
/*
Model for coordinate on a map. Uses quaternion for angles.
Y
^
|
|
0-----> X

*/

class Point {

    private float x = 0; // x offset from 0-point
    private float y = 0; // y offset from 0-point
    private float z = 0; // quaternion z
    private float w = 0; // quaternion w

    Point(float x, float y,float z, float w){
        this.x = x;
        this.y = y;
        this.w = w;
        this.z = z;
    }

    float getX() {
        return x;
    }

    float getY() {
        return y;
    }

    float getW() {
        return w;
    }

    float getZ() {
        return z;
    }
}

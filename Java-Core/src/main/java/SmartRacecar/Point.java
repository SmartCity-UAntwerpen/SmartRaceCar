package SmartRacecar;

class Point {

    private float x = 0;
    private float y = 0;
    private float w = 0;
    private float z = 0;

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

    public float getW() {
        return w;
    }


    public float getZ() {
        return z;
    }


    void setPoint(float x,float y, float z, float w){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
}

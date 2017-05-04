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

    void setX(float x) {
        this.x = x;
    }

    float getY() {
        return y;
    }

    void setY(float y) {
        this.y = y;
    }

    public float getW() {
        return w;
    }

    public void setW(float w) {
        this.w = w;
    }

    public float getZ() {
        return z;
    }

    public void setZ(float z) {
        this.z = z;
    }

    void setPoint(float x,float y){
        this.x = x;
        this.y = y;
    }
}

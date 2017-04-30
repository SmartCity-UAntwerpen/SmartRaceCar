package SmartRacecar;

class Point {

    private float x = 0;
    private float y = 0;

    Point(float x, float y){
        this.x=x;
        this.y=y;
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

    void setPoint(float x,float y){
        this.x = x;
        this.y = y;
    }
}

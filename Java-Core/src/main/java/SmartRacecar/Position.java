package SmartRacecar;

/**
 * Created by Wouter on 27/04/2017.
 */
public class Position {

    float x = 0;
    float y = 0;

    public Position(float x, float y){
        this.x=x;
        this.y=y;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void setPosition(float x,float y){
        this.x = x;
        this.y = y;
    }
}

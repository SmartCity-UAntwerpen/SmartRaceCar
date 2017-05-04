package SmartRacecar;

/**
 * Created by Wouter on 4/05/2017.
 */
class Drive {

    private float steer;
    private float throttle;

    Drive(float steer,float throttle){
        this.steer = steer;
        this.throttle = throttle;
    }

    public float getSteer() {
        return steer;
    }

    public void setSteer(float steer) {
        this.steer = steer;
    }

    public float getThrottle() {
        return throttle;
    }

    public void setThrottle(float throttle) {
        this.throttle = throttle;
    }
}

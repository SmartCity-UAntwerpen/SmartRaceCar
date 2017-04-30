package SmartRacecar;

/**
 * Created by Wouter on 27/04/2017.
 */
public class WayPoint extends Position {

    int ID = 0;
    int weight = 0;

    public WayPoint(int ID,float x, float y, int weight) {
        super(x, y);
        this.ID = ID;
        this.weight = weight;
    }

    public int getID() {
        return ID;
    }

    public void setID(int ID) {
        this.ID = ID;
    }

    public int getWeight() {
        return weight;
    }

    public void setWeight(int weight) {
        this.weight = weight;
    }
}

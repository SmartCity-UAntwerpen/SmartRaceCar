package SmartRacecar;

class WayPoint extends Point {

    private int ID = 0;
    private int weight = 0;

    WayPoint(int ID,float x, float y, int weight) {
        super(x, y);
        this.ID = ID;
        this.weight = weight;
    }

    int getID() {
        return ID;
    }

    void setID(int ID) {
        this.ID = ID;
    }

    int getWeight() {
        return weight;
    }

    void setWeight(int weight) {
        this.weight = weight;
    }
}

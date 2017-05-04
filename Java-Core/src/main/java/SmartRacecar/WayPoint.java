package SmartRacecar;

class WayPoint extends Point {

    private int ID = 0;
    private int weight = 0;

    WayPoint(int ID,float x, float y,float z, float w, int weight) {
        super(x, y, z, w);
        this.ID = ID;
        this.weight = weight;
    }

    int getID() {
        return ID;
    }
}

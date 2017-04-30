package SmartRacecar;

class Map {

    private String name = "";
    private Point startPoint = new Point(0, 0);
    private float meterPerPixel = 0;

    Map(String name, Point startPoint, float meterPerPixel) {
        this.name = name;
        this.startPoint = startPoint;
        this.meterPerPixel = meterPerPixel;
    }

    String getName() {
        return name;
    }

    void setName(String name) {
        this.name = name;
    }

    Point getStartPoint() {
        return startPoint;
    }

    void setStartPoint(Point startPoint) {
        this.startPoint = startPoint;
    }

    float getMeterPerPixel() {
        return meterPerPixel;
    }

    void setMeterPerPixel(float meterPerPixel) {
        this.meterPerPixel = meterPerPixel;
    }

    void setStartPoint(int x, int y) {
        this.startPoint.setX(x);
        this.startPoint.setY(y);
    }
}

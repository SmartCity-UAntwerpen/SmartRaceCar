package SmartRacecar;

class Map {

    private String name = "";
    private float meterPerPixel = 0;

    Map(String name, float meterPerPixel) {
        this.name = name;
        this.meterPerPixel = meterPerPixel;
    }

    String getName() {
        return name;
    }

    void setName(String name) {
        this.name = name;
    }

    float getMeterPerPixel() {
        return meterPerPixel;
    }

    void setMeterPerPixel(float meterPerPixel) {
        this.meterPerPixel = meterPerPixel;
    }

    public class Entry {
    }
}

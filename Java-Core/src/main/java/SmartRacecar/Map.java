package SmartRacecar;

//Model of a offline kept map.
public class Map {

    private String name = ""; //Name of the mapfile.
    private float meterPerPixel = 0; //Sets how many meters there are per pixel to set the correct scale of the map.

    public Map(String name, float meterPerPixel) {
        this.name = name;
        this.meterPerPixel = meterPerPixel;
    }
}

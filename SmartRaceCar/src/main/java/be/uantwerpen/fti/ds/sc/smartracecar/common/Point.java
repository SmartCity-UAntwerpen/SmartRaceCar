package be.uantwerpen.fti.ds.sc.smartracecar.common;

/**
 * Model that describes a coordinate on a map. Uses X and Y and quaternion in the Z and W.
 */
public class Point {

    private float x = 0; // X offset from 0-point
    private float y = 0; // Y offset from 0-point
    private float z = 0; // quaternion Z
    private float w = 0; // quaternion W

    /**
     * Model that describes a coordinate on a map. Uses X and Y and quaternion in the Z and W.
     *
     * @param x X offset from 0-point.
     * @param y Y offset from 0-point.
     * @param z Quaternion Z.
     * @param w Quaternion W.
     */
    public Point(float x, float y, float z, float w){
        this.x = x;
        this.y = y;
        this.w = w;
        this.z = z;
    }

    /**
     * Get the X value of the coordinate.
     *
     * @return The X value of the coordinate.
     */
    public float getX() {
        return x;
    }

    /**
     * Get the Y value of the coordinate.
     *
     * @return The Y value of the coordinate.
     */
    public float getY() {
        return y;
    }

    /**
     * Get the W value of the coordinate.
     *
     * @return The W value of the coordinate.
     */
    public float getW() {
        return w;
    }

    /**
     * Get the Z value of the coordinate.
     *
     * @return The Z value of the coordinate.
     */
    public float getZ() {
        return z;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Point point = (Point) o;

        if (Float.compare(point.x, x) != 0) return false;
        if (Float.compare(point.y, y) != 0) return false;
        if (Float.compare(point.z, z) != 0) return false;
        return Float.compare(point.w, w) == 0;
    }
}

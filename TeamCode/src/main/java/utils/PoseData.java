package utils;

public class PoseData {
    public double x;
    public double y;
    public double h;
    public Waypoint desc;

    public PoseData (double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.desc = Waypoint.UNKNOWN;
    }

    public PoseData (double x, double y, double h, Waypoint wayPoint) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.desc = wayPoint;
    }
}

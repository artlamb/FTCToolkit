package utils;

public class PoseData {
    public double x;
    public double y;
    public double h;
    public double speed;
    public Waypoint desc;

    public PoseData (double x, double y, double h, double speed, Waypoint wayPoint) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.speed = speed;
        this.desc = wayPoint;
    }

    public PoseData (double x, double y, double h) {
        this(x, y, h, 0, Waypoint.UNKNOWN);
    }

    public PoseData (double x, double y, double h, Waypoint wayPoint) {
        this(x, y, h, 0, wayPoint);
    }

    public PoseData (double x, double y, double h, double speed) {
        this(x, y, h, speed, Waypoint.UNKNOWN);
    }
}

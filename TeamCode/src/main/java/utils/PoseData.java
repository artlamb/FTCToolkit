package utils;


import test.PathTest;

public class PoseData {
    public double x;
    public double y;
    public double h;
    public String desc;

    public enum WayPoint {
        START, WAYPOINT_1, WAYPOINT_2, PARK;

        public static String name(WayPoint state) {
            return values()[state.ordinal()].toString();
        }
    }

    public PoseData (double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public PoseData (double x, double y, double h, String desc) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.desc = desc;
    }

    public PoseData (double x, double y, double h, WayPoint wayPoint) {
        this(x, y, h, wayPoint.toString());
    }
}

package utils;

import kotlin.jvm.JvmField;

public class PoseData {
    public double x;
    public double y;
    public double h;
    public String desc;


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

}

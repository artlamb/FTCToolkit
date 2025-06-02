package utils;

import kotlin.jvm.JvmField;

public class PoseData {
    @JvmField public double x;
    @JvmField public double y;
    @JvmField public double h;

    public PoseData (double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
}

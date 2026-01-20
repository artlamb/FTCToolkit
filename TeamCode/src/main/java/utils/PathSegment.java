package utils;

import androidx.annotation.NonNull;

import java.util.Locale;

public class PathSegment {

    public Pose pose;
    public double speed;
    public double tolerance;


    public PathSegment(Pose pose, double speed) {
        this.pose = pose;
        this.speed = speed;
        this.tolerance = 0;
    }

    public PathSegment(Pose pose, double speed, double tolerance) {
        this.pose = pose;
        this.speed = speed;
        this.tolerance = tolerance;
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(Locale.US, "x %5.1f  y %5.1f  heading %5.1f",
                this.pose.getX(), this.pose.getY(), Math.toDegrees(this.pose.getHeading()));
    }
}
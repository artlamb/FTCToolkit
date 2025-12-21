package utils;

import androidx.annotation.NonNull;

import java.util.Locale;

public class PathSegment {

    public Pose pose;
    public double speed;

    public PathSegment(Pose pose, double speed) {
        this.pose = pose;
        this.speed = speed;
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(Locale.US, "x %5.1f  y %5.1f  heading %5.1f",
                this.pose.getX(), this.pose.getY(), Math.toDegrees(this.pose.getHeading()));
    }
}
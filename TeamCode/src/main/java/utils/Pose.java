package utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class Pose extends Pose2D {

    public Pose() {
        super(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    }

    public Pose(double x, double y, double h) {
        super(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, h);
    }

    public Pose(double tileX, double tileY, double offsetX, double offsetY, double heading)  {
        super(DistanceUnit.INCH,
                tileX * 23.5 + offsetX,
                tileY * 23.5 + offsetY,
                AngleUnit.RADIANS,
                heading);
        //assert (heading >= 0 && heading <= 2 * Math.PI);
    }

    /**
     * This returns the x value.
     *
     * @return returns the x value in inches
     */
    public double getX() {
        return super.getX(DistanceUnit.INCH);
    }

    /**
     * This returns the y value.
     *
     * @return returns the y value in inches
     */
    public double getY() {
        return super.getY(DistanceUnit.INCH);
    }

    /**
     * This returns the heading value.
     *
     * @return returns the heading value in radians
     */
    public double getHeading() {
        return super.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Add the first Pose to the second Pose and returns the result as a Pose.
     *
     * @param one the first Pose.
     * @param two the second Pose.
     * @return returns the sum of the two Pose.
     */
    public static Pose add(Pose one, Pose two) {
        double x = one.getX(DistanceUnit.INCH) + two.getX(DistanceUnit.INCH);
        double y = one.getY(DistanceUnit.INCH) + two.getY(DistanceUnit.INCH);
        double heading = one.getHeading(AngleUnit.RADIANS) + two.getHeading(AngleUnit.RADIANS);
        return new Pose(x, y, heading);
    }
    /**
     * This subtracts the second Pose from the first Pose and returns the result as a Pose.
     * Do note that order matters here.
     *
     * @param one the first Pose.
     * @param two the second Pose.
     * @return returns the difference of the two Pose.
     */
    public static Pose subtract(Pose one, Pose two) {
        double x = one.getX(DistanceUnit.INCH) - two.getX(DistanceUnit.INCH);
        double y = one.getY(DistanceUnit.INCH) - two.getY(DistanceUnit.INCH);
        double heading = one.getHeading(AngleUnit.RADIANS) - two.getHeading(AngleUnit.RADIANS);
        return new Pose(x, y, heading);
    }
    /**
     * This rotates the given pose by the given theta,
     *
     * @param pose the Pose to rotate.
     * @param theta the angle to rotate by.
     * @param rotateHeading whether to adjust the Pose heading too.
     * @return the rotated Pose.
     */
    public static Pose rotatePose(Pose pose, double theta, boolean rotateHeading) {
        double x = pose.getX() * Math.cos(theta) - pose.getY() * Math.sin(theta);
        double y = pose.getX() * Math.sin(theta) + pose.getY() * Math.cos(theta);
        double heading = rotateHeading ? normalizeAngle(pose.getHeading() + theta) : pose.getHeading();

        return new Pose(x, y, heading);
    }

    /**
     * This normalizes an angle to be between 0 and 2 pi radians, inclusive.
     * <p>
     * IMPORTANT NOTE: This method operates in radians.
     *
     * @param angleRadians the angle to be normalized.
     * @return returns the normalized angle.
     */
    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians;
        while (angle < 0) angle += 2 * Math.PI;
        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    @Override
    @NonNull
    public String toString() {
        return String.format(Locale.US, "x %5.1f  y %5.1f  heading %5.1f",
                this.getX(), this.getY(), Math.toDegrees(this.getHeading()));
    }
}
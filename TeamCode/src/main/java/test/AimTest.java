package test;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

import common.Logger;
import common.Settings;

/**
 * Calculating the trajectory of a thrown ball involves applying physics principles. Here's a Java function that does this,
 * making some simplifying assumptions common for basic trajectory calculations.
 * <br><br>
 * Assumptions:
 * <br>No Air Resistance: This is the most significant simplification. Air resistance makes real-world trajectory
 * calculations much more complex.
 * <br>Constant Gravity: We assume a constant gravitational acceleration (g).
 * <br>Point Mass: The ball is treated as a point mass, ignoring its size or spin.
 * <br>2D Trajectory: We'll calculate the trajectory in a 2D plane (horizontal x and vertical y).
 * <br><br>Physics Formulas Used:
 * <br>Initial Velocity Components:
 * <br>velocityX = initialVelocity * cos(launchAngle)
 * <br>velocityY = initialVelocity * sin(launchAngle)
 *
 * <br>Position at time t:
 * <br>x(t) = initialX + velocityX * t
 * <br>y(t) = initialY + velocityY * t - 0.5 * g * t^2
 * <br>Time of Flight (to reach the same initial height, or a specific target height): Can be derived from the y(t)
 * equation.
 * <br>Maximum Height: Occurs when the vertical component of velocity becomes zero.
 * <br>Range: Horizontal distance traveled until the ball returns to its initial launch height or hits the ground.
 */
@TeleOp(name="SettingsTest", group="Test")

public class AimTest extends LinearOpMode {

    public static final double GRAVITY = 9.80665; // m/s^2, standard gravity

    // Simple class to represent a 2D point
    public static class Point {
        public final double x;
        public final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        @NonNull
        @SuppressLint("DefaultLocale")
        public String toString() {
            return String.format("(%.2f, %.2f)", x, y);
        }
    }

    // Class to store a point in the trajectory along with the time
    public static class TrajectoryPoint extends Point {
        public final double time;

        public TrajectoryPoint(double x, double y, double time) {
            super(x, y);
            this.time = time;
        }

        @Override
        @NonNull
        @SuppressLint("DefaultLocale")
        public String toString() {
            return String.format("Time: %.2fs, Position: %s", time, super.toString());
        }
    }

    /**
     * Calculates the trajectory of a thrown ball.
     *
     * @param initialVelocity    Magnitude of the initial velocity (m/s).
     * @param launchAngleDegrees Angle of launch in degrees from the horizontal.
     * @param initialPosition    The starting (x, y) position of the ball (m).
     * @param timeStep           The time interval for calculating points in the trajectory (s).
     * @param maxTime            The maximum simulation time for the trajectory (s).
     *                           Or, the trajectory stops if the ball hits y=0 (ground from initialPosition.y >=0).
     * @return A list of TrajectoryPoint objects representing the path of the ball.
     */
    public static List<TrajectoryPoint> calculateTrajectory(
            double initialVelocity,
            double launchAngleDegrees,
            Point initialPosition,
            double timeStep,
            double maxTime) {

        if (initialVelocity < 0) {
            throw new IllegalArgumentException("Initial velocity cannot be negative.");
        }
        if (timeStep <= 0) {
            throw new IllegalArgumentException("Time step must be positive.");
        }
        if (maxTime <= 0) {
            throw new IllegalArgumentException("Max time must be positive.");
        }

        List<TrajectoryPoint> trajectory = new ArrayList<>();

        double launchAngleRadians = Math.toRadians(launchAngleDegrees);

        // Initial velocity components
        double velocityX = initialVelocity * Math.cos(launchAngleRadians);
        double velocityY = initialVelocity * Math.sin(launchAngleRadians);

        double currentTime = 0;
        double currentX = initialPosition.x;
        double currentY = initialPosition.y;

        trajectory.add(new TrajectoryPoint(currentX, currentY, currentTime));

        while (currentTime < maxTime) {
            currentTime += timeStep;

            // Calculate new position
            currentX = initialPosition.x + velocityX * currentTime;
            currentY = initialPosition.y + velocityY * currentTime - 0.5 * GRAVITY * Math.pow(currentTime, 2);

            // Stop if ball hits the ground (or goes below initial y if initial y is ground)
            // This condition assumes the ground is at y=0.
            // If the initial position y is above 0, it will stop when y <= 0.
            // If initial position y is 0, it will stop when y becomes negative on the next step.
            if (currentY < 0 && initialPosition.y >= 0) {
                // Optional: Calculate the exact time of impact with y=0 for more precision
                // This involves solving y(t) = 0 for t, which is a quadratic equation.
                // For simplicity here, we just add the last point before it would go negative significantly.
                // If the previous point was above ground and this one is below,
                // we could interpolate to find the impact point.
                // For now, we'll just add the point and break.
                trajectory.add(new TrajectoryPoint(currentX, currentY, currentTime)); // Add the point where it's ~ at/below ground
                break;
            }

            trajectory.add(new TrajectoryPoint(currentX, currentY, currentTime));

            // If a specific target condition is met, you could also break here
            // e.g., if (currentX >= targetDistance) break;
        }

        return trajectory;
    }

    /**
     * Calculates the time of flight until the ball returns to its initial launch height.
     * Assumes initialPosition.y is the reference height.
     *
     * @param initialVelocity    m/s
     * @param launchAngleDegrees degrees
     * @return time in seconds
     */
    public static double timeOfFlightToInitialHeight(double initialVelocity, double launchAngleDegrees) {
        if (initialVelocity < 0) return 0;
        double launchAngleRadians = Math.toRadians(launchAngleDegrees);
        double velocityY = initialVelocity * Math.sin(launchAngleRadians);
        if (velocityY <= 0 && launchAngleDegrees <= 0)
            return 0; // Launched downwards or horizontally from ground
        return (2 * velocityY) / GRAVITY;
    }

    /**
     * Calculates the maximum height reached by the ball relative to its launch position.
     *
     * @param initialVelocity    m/s
     * @param launchAngleDegrees degrees
     * @return maximum height in meters
     */
    public static double maxFlightHeight(double initialVelocity, double launchAngleDegrees) {
        if (initialVelocity < 0) return 0;
        double launchAngleRadians = Math.toRadians(launchAngleDegrees);
        double velocityY = initialVelocity * Math.sin(launchAngleRadians);
        return (velocityY * velocityY) / (2 * GRAVITY);
    }

    /**
     * Calculates the horizontal range until the ball returns to its initial launch height.
     *
     * @param initialVelocity    m/s
     * @param launchAngleDegrees degrees
     * @return range in meters
     */
    public static double rangeToInitialHeight(double initialVelocity, double launchAngleDegrees) {
        if (initialVelocity < 0) return 0;
        double launchAngleRadians = Math.toRadians(launchAngleDegrees);
        double velocityX = initialVelocity * Math.cos(launchAngleRadians);
        double timeToInitialHeight = timeOfFlightToInitialHeight(initialVelocity, launchAngleDegrees);
        return velocityX * timeToInitialHeight;
    }

    @Override
    public void runOpMode() {

        double v0 = 20.0; // Initial velocity in m/s
        double angle = 45.0; // Launch angle in degrees
        Point startPos = new Point(0, 0); // Starting from origin
        double dt = 0.1; // Time step in seconds
        double tMax = 5.0; // Max simulation time

        List<TrajectoryPoint> trajectory = calculateTrajectory(v0, angle, startPos, dt, tMax);

        System.out.println("Calculating trajectory for:");
        System.out.println("Initial Velocity: " + v0 + " m/s");
        System.out.println("Launch Angle: " + angle + " degrees");
        System.out.println("Initial Position: " + startPos);


        telemetry.addData("Status", "Press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            sleep(10);
        }
    }

}
package common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * <h3>FTC Robot Turret Project Specifications</h3>
 *
 * <p>
 * This class implements a target-locking system for a gear-driven turret based on the following
 * requirements:
 * </p>
 *
 * <b>Hardware Configuration</b>
 * <ul>
 *   <li><b>Actuator:</b> GoBILDA 5-turn servo (providing a total of 1800&deg; of rotation).</li>
 *   <li><b>Gearing:</b> Gear-driven assembly with a {@code 2.75:1} reduction ratio.</li>
 *   <li><b>Calibration:</b> A servo position of {@code 0.5} is defined as the turret
 *       facing forward on the robot chassis.</li>
 * </ul>
 *
 * <b>Localization & Navigation</b>
 * <ul>
 *   <li><b>Odometer:</b> Pinpoint Odometer system providing real-time X/Y coordinates
 *       and robot heading.</li>
 *   <li><b>Coordinate System:</b> Cartesian coordinate system with {@code (0, 0)}
 *       representing the center of the 12ft x 12ft (144" x 144") FTC field.</li>
 * </ul>
 * </p>
 * <ul>
 *   <li><b>Alliance Logic:</b> The turret targets goals specific to the selected alliance.
 *       Red Alliance goals are on the left (X &lt; 0), Blue on the right (X &gt; 0).</li>
 *   <li><b>Quadrant Logic:</b> Targets the "Top Goal" when the robot is in the top two quadrants (Y &gt; 0)
 *       and the "Bottom Goal" when in the bottom two quadrants (Y &lt; 0).</li>
 * </ul><br>
 *
 * <b>Functional Objective</b>
 * <p>
 * The primary goal of this system is to ensure the turret end-effector is consistently
 * oriented toward a fixed goal position. The class must dynamically calculate the
 * correct servo position by comparing the robot's field-relative position to the target
 * coordinate, accounting for the robot's current heading.
 * <h3>Turret Tracking Logic</h3>
 * <p>
 * This class implements a gear-driven assembly tracking system using the following logic:
 * </p>
 *
 * <b>1. Coordinate Geometry</b>
 * <ul>
 *   <li>Calculates the vector from the robot's current position {@code (robotX, robotY)}
 *       to the goal's fixed field coordinates {@code (GOAL_X, GOAL_Y)}.</li>
 *   <li>The absolute world-angle is determined using {@link Math#atan2(double, double)}.</li>
 * </ul>
 *
 * <b>2. Heading Reference Transformation</b>
 * <ul>
 *   <li>To find the turret's position relative to the robot chassis, the robot's current
 *       heading is subtracted from the absolute goal angle.</li>
 *   <li>The resulting value is normalized to the range {@code [-PI, PI]} to ensure
 *       the turret takes the shortest path to the target.</li>
 * </ul>
 *
 * <b>3. Physical Scaling and Gearing</b>
 * <ul>
 *   <li><b>Servo Range:</b> The GoBILDA 5-turn servo provides a total of 1800 degrees of rotation.</li>
 *   <li><b>Gear Reduction:</b> The 2.75:1 ratio reduces the turret's physical range to
 *       approximately 654.5 degrees ({@code 1800 / 2.75}).</li>
 *   <li><b>Zeroing:</b> The servo range (0.0 to 1.0) is mapped such that 0.5 is the
 *       "home" position (facing forward).</li>
 * </ul>
 *
 * <b>4. Servo Command Calculation</b>
 * <p>
 * The final position is calculated by adding the required rotation offset to the 0.5
 * home position, scaled by the {@code SERVO_TICKS_PER_DEGREE} constant, and clamped
 * between 0.0 and 1.0 to prevent hardware strain.
 * </p><br>
 * <ul>
 *   <li><b>Alliance Logic:</b> The turret targets goals specific to the selected alliance.
 *       Red Alliance goals are on the left (X &lt; 0), Blue on the right (X &gt; 0).</li>
 *   <li><b>Quadrant Logic:</b> Targets the "Top Goal" when the robot is in the top two quadrants (Y &gt; 0)
 *       and the "Bottom Goal" when in the bottom two quadrants (Y &lt; 0).</li>
 * </ul>
 */
public class Turret {

    public enum Alliance {
        RED, BLUE
    }

    private final Servo turretServo;
    private Alliance currentAlliance = Alliance.RED; // Default

    // Goal Coordinates (Inches)
    private static final double RED_GOAL_X = -72.0;
    private static final double BLUE_GOAL_X = 72.0;
    private static final double TOP_GOAL_Y = 36.0;
    private static final double BOTTOM_GOAL_Y = -36.0;

    // Hardware Constants
    private static final double SERVO_TOTAL_TURNS = 5.0;
    private static final double GEAR_RATIO = 2.75;
    private static final double TURRET_RANGE_DEGREES = (SERVO_TOTAL_TURNS * 360.0) / GEAR_RATIO;
    private static final double SERVO_TICKS_PER_DEGREE = 1.0 / TURRET_RANGE_DEGREES;

    public Turret(HardwareMap hardwareMap) {
        this.turretServo = hardwareMap.get(Servo.class, "turretServo");
    }

    /**
     * Sets the active alliance for target filtering.
     * @param alliance The alliance the robot is currently on.
     */
    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    /**
     * Updates the turret to point at the correct goal based on alliance and quadrant.
     *
     * @param robotX       Current X from Pinpoint (inches)
     * @param robotY       Current Y from Pinpoint (inches)
     * @param robotHeading Current Heading from Pinpoint (Radians)
     */
    public void update(double robotX, double robotY, double robotHeading) {
        // 1. Determine Target Goal Coordinates
        double targetX = (currentAlliance == Alliance.RED) ? RED_GOAL_X : BLUE_GOAL_X;
        double targetY = (robotY >= 0) ? TOP_GOAL_Y : BOTTOM_GOAL_Y;

        // 2. Calculate absolute angle to the selected goal
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double absoluteAngleToGoal = Math.atan2(deltaY, deltaX);

        // 3. Calculate relative angle and normalize to [-PI, PI]
        double relativeAngleRad = absoluteAngleToGoal - robotHeading;
        while (relativeAngleRad > Math.PI) relativeAngleRad -= 2 * Math.PI;
        while (relativeAngleRad <= -Math.PI) relativeAngleRad += 2 * Math.PI;

        // 4. Map to servo position (0.5 is forward)
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        double targetServoPosition = 0.5 + (relativeAngleDeg * SERVO_TICKS_PER_DEGREE);

        // 5. Apply with safety clamp
        turretServo.setPosition(Math.max(0.0, Math.min(1.0, targetServoPosition)));
    }

    /**
     * Sets the turret to a specific heading relative to the front of the robot.
     *
     * @param relativeDegrees The target angle in degrees.
     *                        0 is forward, positive is counter-clockwise,
     *                        negative is clockwise.
     */
    public void setRelativeHeading(double relativeDegrees) {
        // 0.5 is forward. We add the degrees scaled by our tick constant.
        double targetServoPosition = 0.5 + (relativeDegrees * SERVO_TICKS_PER_DEGREE);

        // Apply with safety clamp to ensure we stay within the 0.0 to 1.0 servo range.
        turretServo.setPosition(Math.max(0.0, Math.min(1.0, targetServoPosition)));
    }
}